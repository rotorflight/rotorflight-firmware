# Rotorflight Glossary

Domain vocabulary for RC helicopters and Rotorflight firmware. Knowing these terms is
essential for understanding what the code is doing and why.

---

## Rotor Mechanics

**Collective pitch** — The pitch angle applied equally to all main rotor blades simultaneously.
Increasing collective increases lift; decreasing it reduces lift or produces negative thrust
(used in autorotation). Controlled by the throttle/collective stick on the transmitter.
In firmware: the primary vertical axis output from the mixer.

**Cyclic pitch** — Variable pitch applied to individual blades as they rotate, causing the
rotor disc to tilt in a direction. Split into:
- **Roll cyclic** (aileron input) — tilts disc left/right
- **Pitch cyclic** (elevator input) — tilts disc forward/back

**Swashplate** — The mechanical interface between the stationary servos and the rotating
rotor head. It tilts (cyclic) and moves up/down (collective). Servo pushrods connect to
its outer ring; pitch links from the rotor head connect to its inner ring.

**CCPM (Cyclic-Collective-Pitch-Mixing)** — A swashplate design where 3 servos (typically
at 120° spacing) together drive both collective and cyclic movements electronically. All
mixing is done in software. This is the standard on all modern collective-pitch helis;
there is no mechanical mixing.

**Pitch link** — The short rod connecting the swashplate to each rotor blade grip. Changes
in swashplate position translate directly into blade pitch angle changes.

**Blade grip** — The hub assembly that holds each rotor blade and allows it to change pitch
angle. Contains bearings for smooth pitch rotation.

**Rotor head** — The complete assembly at the top of the main shaft: blade grips, pitch
links, feathering spindles, and (on flybar helis) the flybar. The swashplate sits below it.

---

## Stabilisation

**Flybar** — A weighted stabiliser bar perpendicular to the rotor blades on traditional
helis. It acts as a mechanical gyro, providing passive attitude stability and mixing its
own motion into blade pitch.

**Flybarless (FBL)** — A helicopter without a mechanical flybar, relying entirely on
electronic sensors and a control loop to maintain stability.

**FBL unit / flight controller** — The electronics that replace the flybar: IMU (gyro +
accelerometer), processor, and servo outputs. Rotorflight *is* the FBL unit.

**Rate mode (Acro mode)** — The flight controller controls rotation *rate* (degrees/sec).
No auto-levelling; the pilot commands how fast the heli rotates. This is the primary mode
for experienced pilots and 3D flying. The inner PID loop always operates in rate mode.

**Angle mode (Self-level)** — The flight controller controls *attitude* (degrees from
level). The heli returns to level when sticks are centred. Uses both gyro and
accelerometer. Implemented as an outer loop on top of rate mode.

**Rescue mode** — Emergency recovery: immediately levels the heli regardless of orientation
and applies pull-up collective to arrest descent, then transitions to a stable hover.
Activated by a switch.

---

## Tail Rotor

**Tail rotor (anti-torque rotor)** — The small rotor on the tail boom that counteracts the
torque reaction from the main rotor. Also provides yaw control (heading control).

**Tail authority** — How much yaw control force the tail rotor can produce. Low authority
causes the heli to pirouette uncontrollably under high-power manoeuvres.

**Tail drive types:**
- **Direct drive (torque tube)** — A rigid carbon or steel shaft runs inside the tail boom.
  Efficient and durable but vulnerable to crash damage.
- **Belt drive** — A toothed belt drives the tail gearbox. More crash-tolerant but higher
  friction losses.

**Pirouette** — Rotation around the yaw axis (vertical axis). A deliberate pirouette is a
3D manoeuvre; an uncontrolled pirouette is a tail failure.

---

## Rotor Speed

**Head speed** — Main rotor RPM. Critical: changes in head speed directly affect cyclic
control authority and stability. Typical values: 1500–2200 RPM (sport), 1200–1600 RPM
(scale). Maintained by the governor.

**Governor** — A closed-loop RPM controller. It measures actual rotor speed and adjusts
the throttle signal to the ESC to hold a target head speed regardless of load (collective
input, wind, manoeuvres). Essential for consistent flight feel. Requires a high-rate RPM
source: a dedicated RPM sensor, or a high-rate RPM signal from the ESC (e.g. DSHOT
bidirectional). ESC telemetry is not suitable due to its low update rate.

**Spool-up** — The controlled acceleration of the rotor from rest to flight head speed.
Must be gradual to avoid gear/belt shock. Managed by the ESC's soft-start and the
governor's ramp rate.

**Spool-down** — The controlled deceleration of the rotor after landing. The governor
manages throttle reduction.

**Autorotation** — Unpowered descent with the main rotor kept spinning by the upward
airflow through the disc. The pilot uses stored rotor inertia to flare and land safely.
Requires negative collective to maintain rotor RPM during descent.

**Bailout** — Aborting an autorotation by rapidly restoring throttle and positive
collective. Rotorflight supports configurable bailout ramp rates.

**RPM sensor** — A sensor that counts motor revolutions for the governor and RPM notch
filter. Two distinct types exist for different use cases:
- **Magnetic sensor (Hall effect)** — Used on combustion engines (nitro/gas), where there
  is no ESC to provide RPM data. A magnet on the flywheel or clutch bell triggers a Hall
  effect sensor on each pass.
- **Electric sensor** — Used on electric helis whose ESC lacks an RPM output (no DSHOT
  bidirectional eRPM, no ESC telemetry). Counts motor commutation pulses electrically.

In both cases the sensor reads motor RPM, which is converted to head speed using the gear
ratio.

---

## Drivetrain

**Gear ratio (main)** — The ratio of motor RPM to head speed. e.g., a 9.5:1 ratio means
the motor spins 9.5× faster than the rotor. Used to convert motor RPM sensor readings to
head speed.

**Gear ratio (tail)** — The ratio between main rotor RPM and tail rotor RPM. The tail
rotor typically spins 4–6× faster than the main rotor.

**Pinion / main gear** — The small drive gear on the motor (pinion) meshes with the large
main gear on the main shaft. Changing the pinion tooth count changes the gear ratio.

**One-way bearing (sprag clutch)** — A bearing in the rotor head that allows the rotor to
freewheel faster than the motor (needed for autorotation) but locks for powered flight.

---

## Electronic Speed Controller (ESC)

**ESC** — Controls motor speed by varying the power delivered to the brushless motor.
Receives a throttle signal from the flight controller and drives the three motor phases.

**BEC (Battery Eliminator Circuit)** — Voltage regulator inside the ESC (or external) that
powers the flight controller and servos from the main flight battery. Eliminates a
separate receiver battery.

**Soft start** — ESC feature that limits the rate of motor acceleration during spool-up to
protect the drivetrain.

**ESC protocols:**
- **PWM** — Analogue pulse-width signal, 1000–2000 µs. Slowest, widest compatibility.
- **DSHOT** — Digital protocol (DSHOT150/300/600/1200). No calibration needed; supports
  bidirectional telemetry (eRPM). Preferred for Rotorflight when ESC supports it.
- **OneShot125/42, Multishot** — Digital protocols faster than PWM but older than DSHOT.

**ESC telemetry** — A separate serial wire from the ESC to the flight controller carrying
low-rate monitoring data: current, voltage, temperature, and RPM. Update rate is too low
for the governor or RPM notch filter — it is suitable for logging, OSD display, and
battery monitoring only. Supported brands in Rotorflight include BLHeli32, Hobbywing, YGE,
Scorpion, Kontronik, OMP, ZTW, APD.

**eRPM (electrical RPM)** — Motor electrical RPM = mechanical RPM × (number of pole
pairs). Must be divided by pole-pair count to get actual shaft RPM, then divided by gear
ratio to get head speed.

---

## RC Link Protocols

**CRSF (Crossfire Serial Protocol)** — Full-duplex serial protocol by TBS. Carries RC
channels to the FC and telemetry back to the transmitter on a single UART. Used by TBS
Crossfire and all ExpressLRS systems. 420 kBaud.

**ELRS (ExpressLRS)** — Open-source long-range RC protocol, uses CRSF framing. Supports
900 MHz and 2.4 GHz. Growing standard in the community.

**S.Bus** — Futaba/FrSky inverted serial protocol, 100 kBaud, 16 channels. Control only,
no telemetry. Widely supported; still common on older hardware.

**FBUS (F.Port 2.0)** — FrSky's current single-wire, half-duplex serial protocol combining
up to 24 RC channels and bidirectional S.Port telemetry. Runs at 460,800 baud (inverted
UART, 8N1). Supports a multi-drop daisy-chain bus and OTA firmware updates via the ACCESS
system. Signal inversion requires an external inverter on F4 MCUs; F7/H7 handle it in
hardware. Supersedes F.Port (115,200 baud, no multi-device support).

**IBUS** — FlySky serial protocol. Control only.

**GHOST** — ImmersionRC full-duplex serial protocol.

**CPPM** — Legacy single-wire pulse train, all channels multiplexed. Slow update rate.

---

## Mixer

**Mixer** — The firmware component that translates pilot stick inputs (roll, pitch, yaw,
collective) into individual servo and motor commands. For helicopters this involves
swashplate geometry, CCPM mixing, and tail rotor mixing.

**Swashplate geometry correction** — Compensation for the non-linear relationship between
servo travel and blade pitch angle caused by the circular arc of the pushrod. Applied in
firmware via an arcsin correction.

**Collective-to-tail feed-forward** — Automatic tail rotor compensation that anticipates
the torque change from a collective input, reducing yaw disturbance before the gyro
reacts.

**Differential thrust yaw** — An alternative yaw control method (for co-axial or
dual-motor designs) that uses differential motor speed instead of a tail rotor.

---

## Filters

**RPM notch filter** — A narrow band-stop filter tuned to the fundamental frequency and
harmonics of the main and tail rotors. Removes vibration from the gyro signal without
adding significant latency. Requires a high-rate RPM source: a dedicated RPM sensor, or
a high-rate RPM signal from the ESC (e.g. DSHOT bidirectional). ESC telemetry is not
suitable due to its low update rate.

**Low-pass filter (LPF)** — Removes high-frequency noise from sensor signals at the cost
of phase lag. Used on gyro and D-term.

**Notch filter** — Removes a specific narrow frequency band. Used for motor/rotor noise
that has a known, fixed frequency.

**Dynamic notch filter** — Automatically tracks vibration frequency peaks and places notch
filters on them. Useful for noise that shifts with RPM.

---

## PID Tuning

**P (Proportional)** — Correction proportional to the current error. Too high → oscillation.

**I (Integral)** — Correction proportional to accumulated error over time. Removes steady-
state drift. Too high → slow oscillation / wash-out.

**D (Derivative)** — Correction proportional to the rate of change of error. Damps
overshoot. Too high → high-frequency oscillation / motor noise.

**FF (Feed-forward)** — Open-loop term added proportional to the stick command.
Improves response without waiting for error to build up. Reduces P/I requirement.

**Error limit** — Caps the maximum PID error to prevent integrator wind-up and excessive
corrections during large attitude disturbances.

**I-term relax** — Reduces I accumulation when the pilot commands large stick inputs, to
prevent the I-term from fighting intentional rapid manoeuvres.

**Collective FF** — Feed-forward added to the tail PID proportional to collective input, to
proactively counter torque changes from collective movement.

---

## Profiles & Configuration

**PID profile** — A stored set of PID gains, filter settings, and related tuning parameters.
Multiple profiles can be stored and switched in flight via a transmitter switch.

**Rate profile** — A stored set of RC rate/expo settings defining stick response curves.
Separate from PID profiles; can be mixed and matched.

**Governor profile** — Head speed target and governor gain settings, part of the PID profile.

**Modes (flight modes)** — Assignable conditions activated by transmitter switches:
angle mode, rescue, governor enable, profiles, arming, etc.

---

## Safety

**Arming** — Enabling the flight controller to produce live servo and motor outputs.
Requires explicit arming input to prevent accidental spool-up.

**Failsafe** — The defined behaviour when RC link is lost: typically motor cut and servos
hold last position or move to a safe position.

**Runaway** — Uncommanded spool-up, a dangerous failure mode. Prevented by correct
failsafe configuration and arming logic.

**Throttle protection** — Blocks motor output above idle when the heli is disarmed or
when collective is at an unsafe position during arming.
