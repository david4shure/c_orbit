# C Orbit is an orbital simulation written in C, using raylib

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

# Screenshots

