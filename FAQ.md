# Frequently Asked Questions

## My agent isn't doing anything

The first step is to print out the `AgentState`. This should give you a general idea of what's
wrong. Another tool is to draw the archipelago using `draw_archipelago_debug` in `landmass`, or the
`LandmassDebugPlugin` in `bevy_landmass`.

Some common reasons for nothing happening:

- You have an island without an archipelago (`bevy_landmass`).
- The agent is too far from the nav mesh.
- The target is too far from the nav mesh.

## My agent's state is `AgentNotOnNavMesh`, but it looks like they are standing on it

Make sure your agent position is actually close to the nav mesh (especially vertically). It is
common for the position of an agent to be set to the "center of mass" of an agent. This can be quite
high above the nav mesh. One way to address this is to change the agent's "pivot" to be at its feet.
This way, the position in `landmass` is close to the nav mesh, meaning they are much more likely to
be considered "on the nav mesh".

Another way to address this is to adjust the `PointSampleDistance3d` in the `ArchipelagoOptions`,
specifically the `PointSampleDistance3d::distance_above` and
`PointSampleDistance3d::distance_below`. For example, setting these to `-0.5` and `1.5`
(respectively) would act like the agent is centered at 1 unit below with a vertical range from
-1.5 to -0.5.

## My agent gets stuck going up or down stairs

First, check if this is related to the previous point. Otherwise, this may be a result of not having
a "height mesh". Some navigation mesh generators will generate a low-detail nav mesh for doing the
actual pathfinding (where only navigation-critical geometry is maintained), followed by a
high-detail mesh for accurately representing the geometry (usually the height). It's common for
stairs to result in a very inaccurate low-detail nav mesh (since you can only go up or down the nav
mesh), resulting in an agent on the floor of the stairs actually being considered too far underneath
the nav mesh. A high-detail nav mesh can be used to represent the height of the stairs more
accurately, resulting in the agent being correctly determined to be on the nav mesh.

## My agent goes back and forth on stairs, not making progress

This may occur if your `PointSampleDistance3d` is set too large horizontally. For example, if your
agent is positioned at the center of mass of the character, and the agent goes up some stairs, the
center of mass could be closer to the floor of the level you're trying to climb to than the stairs
you are currently on.

This can be remedied by decreasing the `horizontal_distance`, adjusting the `distance_above` and
`distance_below` to more closely align with the "feet", and tuning the `vertical_preference_ratio`
to more strongly prefer the nav mesh below the agent as opposed to the nav mesh closer to the agent
horizontally.

## My agent slows down around the corners of the nav mesh

This can occur when not setting the velocity of the agent to match the actual physics velocity.
Without the velocity, the agent is incapable of "anticipating" the corners of the nav mesh,
resulting in slowing down too aggresively around the corners.

In `bevy_landmass`, ensure you have a system to update the agent's velocity based on the results of
your physics, and ensure it is actually running!
