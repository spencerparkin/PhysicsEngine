# PhysicsEngine

The aptly named *PhysicsEngine* is, as its namesake suggests, an engine of physics, or "physics engine" in modern parlance.  For now, its primary concern is that of the simulation of the motion of rigid bodies through space.  Each rigid body takes the form of a convex hull.

There appears to be 3 fundamental actions in such simulations.  They are as follows.

 1. Forces
 2. Impulses
 3. Constraints

Forces act at the level of acceleration (linear or angular), impulses act at the level of velocity (linear or angular), and constraints act at the level of position (again, linear or angular.)  Forces may be applied at any time for any reason.  Impulses are applied in response to collisions.  And constraints must be enforced at each tick of the simulation.

This work is primarily based upon the paper "Rigid Body Simulation" by David Baraff.  I will cite more sources here as my research on the topic continues.
