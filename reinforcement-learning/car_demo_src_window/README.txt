
Car Physics Demo by Stephen Thompson
====================================
webpage: http://www.solarflare.org.uk/car_demo/index.html
This is the source code for my car physics demo.

This README file explains a bit about the source code and how to
compile it. For more general information, executables, screenshots
etc., please refer to http://www.solarflare.org.uk/car_demo/index.html.


Visual Studio Solution File
===========================

A solution file for Visual Studio 2008 is included in the "msvc"
folder. The solution is named "featherstone.sln".

If you prefer to read the source code files with some other IDE or
editor, then they can be browsed in the "src" directory.


Compiling
=========

To compile this code you will need to download and install the OGRE
SDK. This can be obtained from: http://www.ogre3d.org/download/

After installing, make sure your OGRE_HOME environment variable is
pointing to the directory where OGRE is installed, otherwise the build
will fail.

Compiling (with Visual Studio) should be straightforward (if
everything is installed correctly): just open the solution file and do
a "Build Solution". 

To run the compiled program, be sure to set "main" as the StartUp
Project. Also, go into the Properties page of the "main" project, and
under "Debugging", set "Working Directory" to
"$(OGRE_HOME)\bin\$(ConfigurationName)". If this is not done, the OGRE
runtime files may not be found correctly (if you get an error about
OgreMain.dll being missing, this is the reason).


Brief Roadmap
=============

The four projects included in the solution file are:

* BulletCollision: This is the collision detection library from
  Bullet. I used it instead of implementing my own collision detection
  routines. 

* featherstone: This contains the basic rigid body physics algorithms,
  including Featherstone's algorithm and the constraint solver. It is
  the core of the project, and is entirely my own work.

* game_world: This co-ordinates the three main game subsystems: the
  physics (provided by the "featherstone" project), the collision
  detection (provided by Bullet) and the graphics (provided by OGRE).
  In effect the GameWorld class is an instance of the Mediator
  pattern.

* main: This is the "top level" of the project and contains all the
  game-specific startup code (including defining the "track layout"
  that the car drives around), the "game logic", the car controls and
  all the user interface code. Basically, if GameWorld is the "game
  engine", then "main" is the game-specific code.

The code also makes use of Eigen
<http://eigen.tuxfamily.org/index.php?title=Main_Page>, a template
library for matrices, vectors and so on. Note I am using version 2 of
Eigen. Eigen is not visible in the Visual Studio project hierarchy
because it is entirely header files; there are no cpp files to be
compiled in.


Copyright
=========

The Car Physics Demo is Copyright (C) 2011 Stephen Thompson.

The Car Physics Demo is free software: you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program (see LICENCE.txt). If not, see
<http://www.gnu.org/licenses/>.


Contact
=======

I can be contacted by email at: stephen@solarflare.org.uk
