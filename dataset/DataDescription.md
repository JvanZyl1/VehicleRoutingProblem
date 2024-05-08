# Description of data folder

The data folder structure is as follows:
* The 3 subfolders named '0.3', '0.5' and '0.7' represent the percentage of customers that can be served by drones (i.e. '0.3' means that only 30% of customers can be solved by drones). This is linked to the maximum payload capacity?
* The file naming inside each subfolder is as follows:
  * for example: '40_20_0.3' means that the dataset contains 40 customers, the vertical distance from the edge of the area to its center is 20km, and only 30% of customers can be served by the drone

Inside each file there are 6 variables:
* 'StringID': represents nodeID
* 'Type': represents node type (d for depot, c for customer)
* 'X': x coordinate of node
* 'Y': y coordinate of node
* 'Demand': demand of each customer
* 'ServiceBy': how customer can be serviced (T for truck D/T for drone or truck)

Additionally, the file called 'Parameter settings of MOVRPDD model.txt' contains the parameters of the model.