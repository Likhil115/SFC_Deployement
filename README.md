# SFC_Deployement
1.Pull this code into your system.
2.To generate the sfc's randomly run sfc_generator as follows:
  2.1. g++ -std=c++17 -g sfc_generator.cpp -o output
  2.2. ./output
  Now enter the size of sfc and no.of sfc's you want it will generate sfc's for you.
3.Then to run the simulation:
  3.1. g++ -std=c++17 -g main.cpp Simulation.cpp sfc.cpp -o sfc_simulator
  3.2. ./sfc_simulator
The code itself takes the average of 20 iterations and shows you the output comparing two algorithms.
   
  
