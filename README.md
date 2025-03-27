# C Orbit is an orbital simulation written in C, using raylib

[![View the demo video with narration](https://img.youtube.com/vi/kRb1Rg4uK-U/0.jpg)](https://www.youtube.com/watch?v=kRb1Rg4uK-U)

# Demo
<video width="640" height="360" src="https://github.com/user-attachments/assets/7dc03f54-1fc5-4ace-9054-d87ab199ae50" controls></video>


# Features
- Full keplerian orbital simulation
    - Orbital position is computed by solving for Kepler's universal anomaly & Lagrange Functions (stumpc, stumps)
    - [Code here](https://github.com/david4shure/c_orbit/blob/master/src/physics/kepler.c#L546)
    - Elliptical, Parabolic, and Hyperbolic orbits are supported.
- Uses rv_to_classical_elements and classical_elements_to_rv to convert between physical states & fully qualitifed Keplerian Orbits
    - [rv_to_classical_elements](https://github.com/david4shure/c_orbit/blob/master/src/physics/kepler.c#L209) & [classical_elements_to_rv](https://github.com/david4shure/c_orbit/blob/master/src/physics/kepler.c#L358)
- Orbital Lines drawing
    - [See here](https://github.com/david4shure/c_orbit/blob/master/src/physics/orbital_lines.c)
- Sphere of Influence Sphere Rendered
- Fully generalized patched conics
- Camera functions to traverse orbital tree (JKHL, basically Vim controls)


# Upcoming features
- Full orbital propagation using numerical integration
    - 2 Body trajectories, to the moon and back for example.
- Load orbital data from simple text files which define arbitrary hierarchies of bodies and their orbital elements
