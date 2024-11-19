# C Orbit is an orbital simulation written in C, using raylib

# Features
- Full keplerian orbital simulation
    - Orbital position is computed by solving for Kepler's universal anomaly & Lagrange Functions (stumpc, stumps)
    - [Code here](https://github.com/david4shure/c_orbit/blob/master/src/physics/kepler.c#L530)
[-](-) Uses rv_to_classical_elements and classical_elements_to_rv to convert between physical states & fully qualitifed Keplerian Orbits
    - [rv_to_classical_elements](https://github.com/david4shure/c_orbit/blob/master/src/physics/kepler.c#L199) & [classical_elements_to_rv](https://github.com/david4shure/c_orbit/blob/master/src/physics/kepler.c#L350)
- Orbital Lines drawing
    - [See here](https://github.com/david4shure/c_orbit/blob/master/src/physics/orbital_lines.c)

# Screenshots


