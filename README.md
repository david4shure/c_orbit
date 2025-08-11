# C Orbit is an orbital simulation written in C, using raylib
<img width="2560" height="1235" alt="Screenshot 2025-08-11 at 2 03 53 PM" src="https://github.com/user-attachments/assets/92bb3e98-000a-4574-82b2-9f291bb64b7b" />

# Narrated Demo

[![View the demo video with narration](https://img.youtube.com/vi/9iFd_e9Pstg/0.jpg)](https://www.youtube.com/watch?v=9iFd_e9Pstg)

## Features

- Full Keplerian orbital simulation
  - Orbital position is computed by solving for Kepler's universal anomaly & Lagrange Functions (`stumpc`, `stumps`)
  - Code [available here](#)
  - Elliptical, Parabolic, and Hyperbolic orbits are supported
- Uses `rv_to_classical_elements` and `classical_elements_to_rv` to convert between physical states & fully qualified Keplerian orbits
- Orbital lines drawing ([example](#))
- Sphere of influence sphere rendered
- Fully generalized patched conics
- Camera functions to traverse orbital tree (`J`, `K`, `H`, `L` – basically Vim-style controls)

## Upcoming Features

- Full orbital propagation using numerical integration
  - 2-body trajectories (e.g., to the Moon and back)
- Load orbital data from simple text files that define arbitrary hierarchies of bodies and their orbital elements
