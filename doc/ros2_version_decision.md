
# ROS2 Version Decision

There are several versions of ROS2 available to choose from. As of the start of this project (9/25),
the following versions were avaliable:

 * Kilted
 * Jazzy (LTS)
 * Iron (EOL)
 * Humble (LTS)
 * Galactic (EOL)
 * Foxy (EOL)
 * Eloquent (EOL)
 * Dashing (EOL)
 * Crystal (EOL)
 * Rolling

To avoid various bugs and issues and ensure that we continually got up to date support, we opted
to use a version that was not in its end-of-life (EOL) phase. This eliminated most of our options.
The code for the previous year was written in Iron, so was mildly unfortunate to rule it out. Given
this however, we were free to choose as new a version as possible. We decided on Jazzy because it is
hopefully more modern than Humble, but also because it is a long-term-stable (LTS) release, meaning we have 
less to worry about in terms of things changing suddenly or issues not being fixed. Kilted was not chosen 
because it is an edge version that will reach EOL very quickly, and Rolling was not chosen because it is a 
version that tracks with the latest updates and as a result is the most unstable. Humble was not
chosen because we had the option for a newer version.

