/*
Title: SAT-2D (OBB)
File Name: VertexShader.glsl
Copyright � 2015
Original authors: Brockton Roth
Written under the supervision of David I. Schwartz, Ph.D., and
supported by a professional development seed grant from the B. Thomas
Golisano College of Computing & Information Sciences
(https://www.rit.edu/gccis) at the Rochester Institute of Technology.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Description:
This is a Separating Axis Theorem test. (Sometimes just called Separating Axis Test.) This is in 2D.
Contains two squares, one that is stationary and one that is moving. They are bounded by OBBs 
(Object-Oriented Bounding Boxes) and when these OBBs collide the moving object "bounces" on the 
x axis (because that is the only direction the object is moving). The algorithm will detect any 
axis of collision, but will not output the axis that was collided (because it doesn't know). Thus, 
we assume x and hardcode in the x axis bounce.
There is a physics timestep such that every update runs at the same delta time, regardless of how fast 
or slow the computer is running. The squares should be the exact same as their OBBs, since they are 
aligned on the same axis.
*/

#version 400 core // Identifies the version of the shader, this line must be on a separate line from the rest of the shader code
 
layout(location = 0) in vec3 in_position;	// Get in a vec3 for position
layout(location = 1) in vec4 in_color;		// Get in a vec4 for color

out vec4 color; // Our vec4 color variable containing r, g, b, a

uniform mat4 MVP; // Our uniform MVP matrix to modify our position values

void main(void)
{
	color = in_color;	// Pass the color through
	gl_Position = MVP * vec4(in_position, 1.0); //w is 1.0, also notice cast to a vec4
}