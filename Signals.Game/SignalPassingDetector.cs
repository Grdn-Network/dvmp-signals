using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using DV.Simulation.Cars;
using DV.Simulation.Controllers;
using DV.ThingTypes;
using Signals.API;
using UnityEngine;

namespace Signals.Game
{
    /// <summary>
    /// Attached as a child GameObject to each signal.
    /// Applies emergency brakes to any train that passes through a signal
    /// whose active aspect has <see cref="Signals.Common.Aspects.AspectBaseDefinition.DisallowPassing"/> set.
    /// </summary>
    /// <remarks>
    /// Only uncoupled end-couplers of a trainset trigger (coupled couplers between cars have
    /// no active collider). This gives exactly two events per trainset pass:
    /// <list type="number">
    ///   <item><b>First coupler</b> (leading end) enters the zone — the signal state is read
    ///         and, if restrictive with the correct approach direction, emergency brakes are
    ///         applied immediately.</item>
    ///   <item><b>Second coupler</b> (trailing end) enters — clears the tracked trainset so
    ///         it can be evaluated again on a subsequent pass.</item>
    /// </list>
    /// Direction is determined by checking which side of the signal the coupler entered from
    /// (dot product of signal forward vs coupler-to-signal offset).
    /// </remarks>
    [RequireComponent(typeof(BoxCollider))]
    internal class SignalPassingDetector : MonoBehaviour
    {
        private const float ColliderWidth = 4f;
        private const float ColliderHeight = 4f;
        private const float ColliderDepth = 2f;

        // Distance along the approach direction (local +Z) to offset the detector
        // ahead of the signal line. This prevents a race condition where the leading
        // bogie enters the protected block (and triggers TrainDetectedAspect) before
        // the passing coupler reaches the zone and reads the signal state.
        private const float ApproachOffset = 4f;

        // Brakes are held until all cars in the trainset drop below this speed (m/s).
        // 1 m/s ≈ 3.6 km/h
        private const float StopSpeedThreshold = 1f;

        private const string CouplerFrontName = "[coupler front]";
        private const string CouplerRearName = "[coupler rear]";
        private static AudioClip? _indusiClip;
        // All live detector instances — used by GrantOverride to cancel enforcement across signals.
        private static readonly List<SignalPassingDetector> _activeDetectors = new List<SignalPassingDetector>();
        // Trainsets that have been granted a one-shot override exemption.
        // Exemption is consumed the first time the trainset would trigger enforcement.
        private static readonly HashSet<Trainset> _exemptedTrainsets = new HashSet<Trainset>();

        private Controllers.BasicSignalController _signal = null!;
        private BoxCollider _collider = null!;
        private Coroutine? _brakeCoroutine;
        // AudioSource currently playing the indusi alarm
        private AudioSource? _audioSource;
        // The trainset currently being braked by this detector instance (null when idle).
        private Trainset? _currentBrakeTrainset;
        // Horn/whistle control we are currently subscribed to for signal override input.
        private HornControl? _subscribedHorn;

        // Trainsets currently being tracked. Present = first coupler has been seen.
        // Cleared when the second coupler enters (so the trainset can trigger again
        // on a subsequent pass) or when the signal returns to a non-restrictive aspect.
        private readonly HashSet<Trainset> _trackedTrainsets = new HashSet<Trainset>();

        internal static SignalPassingDetector Attach(Controllers.BasicSignalController signal)
        {
            var go = new GameObject("SignalPassingDetector");
            go.transform.SetParent(signal.Definition.transform, worldPositionStays: false);
            // Offset toward approaching trains (local +Z = transform.forward = along-track direction).
            // Placing the detector ahead of the signal line ensures the protected block
            // is not yet occupied when the leading coupler is first evaluated.
            go.transform.localPosition = new Vector3(0f, 0f, ApproachOffset);
            go.transform.localRotation = Quaternion.identity;

            // Must be on layer 7 (Train_Big_Collider) so it interacts with trains
            // in the physics collision matrix.
            go.layer = 7;

            var detector = go.AddComponent<SignalPassingDetector>();
            detector._signal = signal;

            var col = go.GetComponent<BoxCollider>();
            col.isTrigger = true;
            col.size = new Vector3(ColliderWidth, ColliderHeight, ColliderDepth);
            col.center = Vector3.zero;
            col.gameObject.AddComponent<TeleportArcPassThrough>();
            detector._collider = col;

            signal.AspectChanged += detector.OnAspectChanged;

            _indusiClip ??= LoadIndusiClip();
            _activeDetectors.Add(detector);

            return detector;
        }

        private void OnDestroy()
        {
            if (_signal != null)
                _signal.AspectChanged -= OnAspectChanged;

            UnsubscribeFromHorn();
            _activeDetectors.Remove(this);
            _trackedTrainsets.Clear();
        }

        private void OnAspectChanged(Aspects.AspectBase? newAspect)
        {
            bool isNowRestrictive = newAspect?.Definition.DisallowPassing ?? false;

            // When the signal returns to a non-restrictive aspect (train cleared the zone),
            // clear all tracked state so the next approaching train starts fresh.
            if (!isNowRestrictive && _trackedTrainsets.Count > 0)
            {
                SignalsMod.LogVerbose($"[Enforcement] Signal '{_signal.Name}' is no longer restrictive — clearing {_trackedTrainsets.Count} tracked trainset(s).");
                _trackedTrainsets.Clear();
            }
        }

        private void OnTriggerEnter(Collider other)
        {
            // Only react to coupler colliders.
            var name = other.name;
            if (name != CouplerFrontName && name != CouplerRearName)
            {
                return;
            }

            // Resolve train car via attached Rigidbody.
            var rb = other.attachedRigidbody;
            if (rb == null) return;
            if (!rb.TryGetComponent<TrainCar>(out var trainCar)) return;

            var trainset = trainCar.trainset;
            if (trainset == null) return;

            SignalsMod.LogVerbose($"[Enforcement] Coupler '{name}' of '{trainCar.ID}' entered zone on signal '{_signal.Name}'.");

            if (!_trackedTrainsets.Add(trainset))
            {
                // Second coupler: clear the trainset so it can trigger again on a subsequent pass.
                _trackedTrainsets.Remove(trainset);
                SignalsMod.LogVerbose($"[Enforcement] Second coupler (trailing end) of trainset — clearing tracking for signal '{_signal.Name}'.");

                // Auto-revert: if the signal is in Manual mode and the setting is enabled,
                // switch it back to Automatic now that the train has fully passed.
                if (SignalsMod.Settings.AutoRevertManualSignals && _signal.Mode == SignalMode.Manual)
                {
                    SignalsMod.LogVerbose($"[Enforcement] Train passed manual signal '{_signal.Name}' — reverting to Automatic.");
                    _signal.SetMode(SignalMode.Automatic);
                }

                return;
            }

            if (!SignalsMod.Settings.EnableSignalEnforcement) return;

            // Only locomotives can be held responsible for passing a signal at danger.
            // When a non-locomotive coupler is the first to enter (e.g. wagons being pushed),
            // skip enforcement. The trainset is already tracked above so the loco's trailing
            // coupler will arrive as the "second coupler" and simply clear the record.
            if (!trainCar.IsLoco)
            {
                SignalsMod.LogVerbose($"[Enforcement] Leading coupler belongs to non-locomotive '{trainCar.ID}' — skipping (locomotive is pushing from behind).");
                return;
            }

            // Direction check: the coupler just entered the zone edge. Its position relative
            // to the signal center tells us which side it came from. The signal's forward
            // points toward the approaching track, so a coupler on that side gives a positive dot.
            Vector3 couplerOffset = other.transform.position - transform.position;
            float dot = Vector3.Dot(transform.forward, couplerOffset.normalized);
            SignalsMod.LogVerbose($"[Enforcement] Direction dot: {dot:F2} (coupler at {other.transform.position}, signal at {transform.position}).");

            if (dot <= 0f)
            {
                SignalsMod.LogVerbose($"[Enforcement] Coupler approached from behind signal '{_signal.Name}' — ignoring.");
                return;
            }

            // Read the signal: check if the current aspect disallows passing.
            if (!CurrentAspectDisallowsPassing())
            {
                SignalsMod.LogVerbose($"[Enforcement] Signal '{_signal.Name}' is not restrictive — trainset may pass freely.");
                return;
            }

            SignalsMod.LogVerbose($"[Enforcement] Signal '{_signal.Name}' read as RESTRICTIVE — triggering enforcement.");

            if (_exemptedTrainsets.Remove(trainset))
            {
                SignalsMod.Log($"[Enforcement] Override active for trainset — passing signal '{_signal.Name}' freely (exemption consumed).");
                return;
            }

            ApplyEmergencyBrake(trainCar);
        }

        private bool CurrentAspectDisallowsPassing()
        {
            var aspects = _signal.AllAspects;
            int index = _signal.CurrentAspectIndex;

            if (aspects == null || index < 0 || index >= aspects.Length) return false;

            return aspects[index].Definition.DisallowPassing;
        }

        private void ApplyEmergencyBrake(TrainCar trainCar)
        {
            bool willBrake = ShouldEnforceForCar(trainCar);
            SignalsMod.Log($"[Enforcement] {trainCar.ID} passed signal '{_signal.Name}' at danger — playing alarm{(willBrake ? " and applying emergency brakes" : string.Empty)} until stopped.");

            if (_brakeCoroutine != null)
                StopCoroutine(_brakeCoroutine);

            // Stop and remove any alarm from a prior enforcement cycle.
            if (_audioSource != null)
            {
                _audioSource.Stop();
                Destroy(_audioSource);
                _audioSource = null;
            }

            // Attach a spatialized audio source directly to the offending car
            // so the alarm moves with the train.
            if (_indusiClip != null)
            {
                _audioSource = trainCar.gameObject.AddComponent<AudioSource>();
                _audioSource.clip = _indusiClip;
                _audioSource.loop = true;
                _audioSource.spatialBlend = 1f;
                _audioSource.volume = SignalsMod.Settings.IndusiVolume;
                _audioSource.Play();
            }

            _currentBrakeTrainset = trainCar.trainset;
            SubscribeToHorn(trainCar);
            _brakeCoroutine = StartCoroutine(SustainBrake(trainCar));
        }

        private IEnumerator SustainBrake(TrainCar triggeringCar)
        {
            var trainset = triggeringCar.trainset;
            bool shouldBrake = ShouldEnforceForCar(triggeringCar);

            while (GetMaxAbsSpeed(trainset) > StopSpeedThreshold)
            {
                if (shouldBrake && trainset != null)
                {
                    // Snapshot the list to avoid InvalidOperationException if the
                    // trainset is uncoupled while we iterate (modifies cars mid-loop).
                    var cars = trainset.cars;
                    if (cars != null)
                    {
                        int count = cars.Count;
                        for (int i = 0; i < count && i < cars.Count; i++)
                        {
                            var car = cars[i];
                            if (car != null) SetBrakePressureZero(car);
                        }
                    }
                }

                // 50 ms between brake applications — still stops the train quickly
                // (10 m/s speed drops in under 5 s) without burning a frame every tick.
                yield return new WaitForSeconds(0.05f);
            }

            // Train has stopped — silence the alarm.
            if (_audioSource != null)
            {
                _audioSource.Stop();
                Destroy(_audioSource);
                _audioSource = null;
            }

            UnsubscribeFromHorn();
            _currentBrakeTrainset = null;
            _brakeCoroutine = null;
        }

        private static float GetMaxAbsSpeed(Trainset? trainset)
        {
            if (trainset == null) return 0f;

            float max = 0f;
            var cars = trainset.cars;
            if (cars == null) return 0f;

            // Iterate by index so a concurrent uncouple (which modifies cars) throws
            // IndexOutOfRange at worst rather than InvalidOperationException mid-foreach.
            for (int i = 0; i < cars.Count; i++)
            {
                var car = cars[i];
                if (car != null)
                    max = Mathf.Max(max, car.GetAbsSpeed());
            }
            return max;
        }

        /// <summary>
        /// Cancels any active enforcement against <paramref name="trainset"/> and grants it
        /// a single-use free pass through the next restrictive signal.
        /// Triggered by the driver sounding the horn/whistle while enforcement is active.
        /// </summary>
        internal static void GrantOverride(Trainset trainset)
        {
            // Cancel any running enforcement across all live detectors.
            foreach (var det in _activeDetectors)
                det.CancelEnforcementIfFor(trainset);

            // Allow the trainset to pass the next restrictive signal without enforcement.
            _exemptedTrainsets.Add(trainset);
            SignalsMod.Log("[Enforcement] Signal override granted — active enforcement cancelled, one free pass through the next restrictive signal.");
        }

        private void CancelEnforcementIfFor(Trainset trainset)
        {
            if (_currentBrakeTrainset != trainset) return;

            if (_brakeCoroutine != null)
            {
                StopCoroutine(_brakeCoroutine);
                _brakeCoroutine = null;
            }

            if (_audioSource != null)
            {
                _audioSource.Stop();
                Destroy(_audioSource);
                _audioSource = null;
            }

            _currentBrakeTrainset = null;
            SignalsMod.Log($"[Enforcement] Enforcement cancelled on signal '{_signal.Name}'.");
        }

        private void SubscribeToHorn(TrainCar trainCar)
        {
            UnsubscribeFromHorn();

            var horn = trainCar.SimController?.controlsOverrider?.Horn;
            if (horn == null) return;

            _subscribedHorn = horn;
            _subscribedHorn.ControlUpdated += OnHornUpdated;
        }

        private void UnsubscribeFromHorn()
        {
            if (_subscribedHorn == null) return;
            _subscribedHorn.ControlUpdated -= OnHornUpdated;
            _subscribedHorn = null;
        }

        private void OnHornUpdated(float value)
        {
            if (_currentBrakeTrainset == null) return;

            var horn = _subscribedHorn;
            if (horn == null) return;

            // Detect an active horn press using the control's neutral position.
            // Diesel/electric: neutral at 0, active when value > 0.
            // Steam: neutral at 0.5, active when value deviates significantly in either direction.
            bool isActive = horn.neutralAt0
                ? value > 0f
                : Mathf.Abs(value - 0.5f) > 0.25f;

            if (isActive)
            {
                SignalsMod.Log("[Enforcement] Horn/whistle detected — granting signal override.");
                GrantOverride(_currentBrakeTrainset);
            }
        }


        private static bool ShouldEnforceForCar(TrainCar car)
        {
            bool isSteam = CarTypes.IsSteamLocomotive(car.carLivery);
            return isSteam
                ? SignalsMod.Settings.EnableSteamEnforcement
                : SignalsMod.Settings.EnableDieselEnforcement;
        }

        private static void SetBrakePressureZero(TrainCar car)
        {
            var brakes = car.brakeSystem;
            if (brakes == null) return;

            brakes.SetBrakePipePressure(0f);
        }

        /// <summary>
        /// Reads the embedded <c>indusi.wav</c> resource, parses its PCM data, and returns
        /// a Unity <see cref="AudioClip"/>. Supports 8-bit and 16-bit PCM WAV files.
        /// Returns <see langword="null"/> and logs a warning on any parse failure.
        /// </summary>
        private static AudioClip? LoadIndusiClip()
        {
            var stream = Assembly.GetExecutingAssembly()
                .GetManifestResourceStream("Signals.Game.indusi.wav");

            if (stream == null)
            {
                SignalsMod.Log("[Enforcement] Could not find embedded resource 'Signals.Game.indusi.wav'.");
                return null;
            }

            byte[] wav;
            using (var ms = new MemoryStream())
            {
                stream.CopyTo(ms);
                wav = ms.ToArray();
            }

            // Walk RIFF subchunks starting at offset 12 (after the 12-byte RIFF/WAVE header).
            int channels = 0, sampleRate = 0, bitsPerSample = 0;
            int dataOffset = 0, dataSize = 0;

            int i = 12;
            while (i < wav.Length - 8)
            {
                string chunkId = System.Text.Encoding.ASCII.GetString(wav, i, 4);
                int chunkSize = BitConverter.ToInt32(wav, i + 4);

                if (chunkId == "fmt ")
                {
                    // audioFormat at i+8 (1 = PCM — we only handle PCM);
                    channels       = BitConverter.ToInt16(wav, i + 10);
                    sampleRate     = BitConverter.ToInt32(wav, i + 12);
                    bitsPerSample  = BitConverter.ToInt16(wav, i + 22);
                }
                else if (chunkId == "data")
                {
                    dataOffset = i + 8;
                    dataSize   = chunkSize;
                    break;
                }

                // Advance to the next chunk; WAV chunks are padded to even byte boundaries.
                i += 8 + ((chunkSize + 1) & ~1);
            }

            if (channels == 0 || sampleRate == 0 || dataOffset == 0)
            {
                SignalsMod.Log("[Enforcement] Failed to parse indusi.wav header.");
                return null;
            }

            if (bitsPerSample != 8 && bitsPerSample != 16)
            {
                SignalsMod.Log($"[Enforcement] Unsupported WAV bit depth: {bitsPerSample}. Only 8-bit and 16-bit PCM are supported.");
                return null;
            }

            int bytesPerSample    = bitsPerSample / 8;
            int totalSamples      = dataSize / bytesPerSample;
            int perChannelSamples = totalSamples / channels;
            float[] samples       = new float[totalSamples];

            if (bitsPerSample == 16)
            {
                for (int s = 0; s < totalSamples; s++)
                    samples[s] = BitConverter.ToInt16(wav, dataOffset + s * 2) / 32768f;
            }
            else // 8-bit PCM is unsigned (0–255), where 128 is silence.
            {
                for (int s = 0; s < totalSamples; s++)
                    samples[s] = (wav[dataOffset + s] - 128) / 128f;
            }

            var clip = AudioClip.Create("indusi", perChannelSamples, channels, sampleRate, stream: false);
            clip.SetData(samples, 0);
            return clip;
        }

    }
}
