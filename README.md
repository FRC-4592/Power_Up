# Power_Up
2018 FRC Game

If using egit, import the repo under import existing eclipse project.
You will have 4 errors:
1) Missing JRE System Library (missing java)
2) Missing WPI Libraries
3) Missing CTRE Libraries
4) Src folder is in each folder name, instead of used as a src folder

Fix for first problem:
1) Right click on the main folder (here it should be Power_Up)
2) Find Build Path
3) Left click on Add Libraries, a window should pop
4) Left click on the library named JRE System Library (or your java library)
5) On the bottom toolbar click finished

Fix for second problem:
1) Right click on the main folder (here it should be Power_Up)
2) Find Build Path
3) Left click on Add External Archives, file explore should open up
4) Navigate to WPI jar 
(normally can be find at c:\Users\(user's name)\wpilib\java\current\lib)
5) Select all jars expect the ones that say source, they are not necessary
6) Left click on open

Fix for third problem:
1) Right click on the main folder (here it should be Power_Up)
2) Find Build Path
3) Left click on Add External Archives, file explore should open up
4) Navigate to CTRE jar 
(normally can be find at c:\Users\(user's name)\wpilib\user\java\lib)
5) Select all jars
6) Left click on open

Fix for fourth problem:
1) Right click on the main folder (here it should be Power_Up)
2) Find Build Path
3) Left click on Configure Inclusion/Exclusion Filters
4) Under exclusion pattern left click add, a window should pop up
5) In the text field write src/
6) Then left click finish
7) Then all folders should have been moved into a single folder src
8) Right click on this new src folder
9) Look for make src folder, and left click
10) It should become the src folder under your project folder