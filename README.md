# KITE
In cyber-physical systems, inertial sensors are the basis for identifying motion states and making actuation decisions. However, extensive studies have proved the vulnerability of those sensors under acoustic transduction attacks, which leverage malicious acoustics to trigger sensor measurement errors. Unfortunately, the threat from such attacks is not assessed properly because of the incomplete investigation on the attack's potential, especially towards multiple-degree-of-freedom systems, e.g., drones. To thoroughly explore the threat of acoustic transduction attacks, we revisit the attack model and design a new yet practical acoustic modulation-based attack, named KITE. Such an attack enables stable and controllable injections, even under frequency offset based distortions that limit the effect of prior attacking approaches. KITE exploits the potential threat of transduction attacks without the need of strengthening attackers’ abilities. Furthermore, we extend the attack surface to multiple-degree-of-freedom systems, which are more widely deployed but ignored by prior work. Our study also covers the scenario of attacking moving targets. By revealing the practical threat from acoustic transduction attacks, we appeal for both the attention to their harm and necessary countermeasures.

# Repository structure
### data
+ This folder contains a table of the attacked devices, the setup of the acoustic transduction attack experiment, and  some experimental results.
### source
+ This folder contains the source code running on the malicious unit(PCB board prototype) for touch-based attacks.
### cases
+ This folder contains two end-to-end attack cases, including the corresponding videos and descriptions.
### The resonant frequency of IMU in COTS devices.md
+ This file records the resonant frequency of 28 COTS devices.
