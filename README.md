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

# Upcoming features
- Full orbital propagation using numerical integration
    - 2 Body trajectories, to the moon and back for example.
- Fully generalized patched conics
- Load orbital data from simple text files which define arbitrary hierarchies of bodies and their orbital elements
    - Camera functions to traverse this hierarchy

# Screenshots

- Parabolic / Hyperbolic Trajectories
<img width="1487" alt="Screenshot 2024-11-19 at 5 41 20 PM" src="https://github.com/user-attachments/assets/135dff61-7895-4d4b-9869-dd329fe5fb26">

- Elliptical Orbits
<img width="1486" alt="Screenshot 2024-11-19 at 5 38 44 PM" src="https://github.com/user-attachments/assets/f089d855-ba2c-4350-869e-b2be968fe9c1">
