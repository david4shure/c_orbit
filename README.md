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
- Fully generalized patched conics

# Upcoming features
- Full orbital propagation using numerical integration
    - 2 Body trajectories, to the moon and back for example.
- Load orbital data from simple text files which define arbitrary hierarchies of bodies and their orbital elements
    - Camera functions to traverse this hierarchy

# Screenshots

- Parabolic / Hyperbolic Trajectories
<img width="1487" alt="Screenshot 2024-11-19 at 5 41 20 PM" src="https://github.com/user-attachments/assets/135dff61-7895-4d4b-9869-dd329fe5fb26">

- Elliptical Orbits
<img width="1179" alt="Screenshot 2024-11-19 at 10 42 03 PM" src="https://github.com/user-attachments/assets/518b1665-c02c-41b1-b7a4-95ef9c0baabe">

- XYZ Axes
<img width="1489" alt="Screenshot 2024-11-20 at 6 06 43 PM" src="https://github.com/user-attachments/assets/293b9015-21a1-42c4-90b6-5e956f83a5e7">
