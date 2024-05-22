\ Model Truck_Routing
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  32787.5 x[Tr1,D0,C1] + 39955 x[Tr1,D0,C2] + 48037.5 x[Tr1,D0,C3]
   + 23180 x[Tr1,D0,C4] + 33245 x[Tr1,D0,C5] + 32787.5 x[Tr1,C1,D0]
   + 9912.5 x[Tr1,C1,C2] + 17995 x[Tr1,C1,C3] + 45902.5 x[Tr1,C1,C4]
   + 4422.5 x[Tr1,C1,C5] + 39955 x[Tr1,C2,D0] + 9912.5 x[Tr1,C2,C1]
   + 8082.5 x[Tr1,C2,C3] + 48800 x[Tr1,C2,C4] + 14335 x[Tr1,C2,C5]
   + 48037.5 x[Tr1,C3,D0] + 17995 x[Tr1,C3,C1] + 8082.5 x[Tr1,C3,C2]
   + 41937.5 x[Tr1,C3,C4] + 22417.5 x[Tr1,C3,C5] + 23180 x[Tr1,C4,D0]
   + 45902.5 x[Tr1,C4,C1] + 48800 x[Tr1,C4,C2] + 41937.5 x[Tr1,C4,C3]
   + 46360 x[Tr1,C4,C5] + 33245 x[Tr1,C5,D0] + 4422.5 x[Tr1,C5,C1]
   + 14335 x[Tr1,C5,C2] + 22417.5 x[Tr1,C5,C3] + 46360 x[Tr1,C5,C4]
   + 32787.5 x[Tr2,D0,C1] + 39955 x[Tr2,D0,C2] + 48037.5 x[Tr2,D0,C3]
   + 23180 x[Tr2,D0,C4] + 33245 x[Tr2,D0,C5] + 32787.5 x[Tr2,C1,D0]
   + 9912.5 x[Tr2,C1,C2] + 17995 x[Tr2,C1,C3] + 45902.5 x[Tr2,C1,C4]
   + 4422.5 x[Tr2,C1,C5] + 39955 x[Tr2,C2,D0] + 9912.5 x[Tr2,C2,C1]
   + 8082.5 x[Tr2,C2,C3] + 48800 x[Tr2,C2,C4] + 14335 x[Tr2,C2,C5]
   + 48037.5 x[Tr2,C3,D0] + 17995 x[Tr2,C3,C1] + 8082.5 x[Tr2,C3,C2]
   + 41937.5 x[Tr2,C3,C4] + 22417.5 x[Tr2,C3,C5] + 23180 x[Tr2,C4,D0]
   + 45902.5 x[Tr2,C4,C1] + 48800 x[Tr2,C4,C2] + 41937.5 x[Tr2,C4,C3]
   + 46360 x[Tr2,C4,C5] + 33245 x[Tr2,C5,D0] + 4422.5 x[Tr2,C5,C1]
   + 14335 x[Tr2,C5,C2] + 22417.5 x[Tr2,C5,C3] + 46360 x[Tr2,C5,C4]
   + 500 y[Tr1] + 500 y[Tr2] + t_max
Subject To
 Customer_C1_visited_once: x[Tr1,D0,C1] + x[Tr1,C2,C1] + x[Tr1,C3,C1]
   + x[Tr1,C4,C1] + x[Tr1,C5,C1] + x[Tr2,D0,C1] + x[Tr2,C2,C1]
   + x[Tr2,C3,C1] + x[Tr2,C4,C1] + x[Tr2,C5,C1] = 1
 Customer_C2_visited_once: x[Tr1,D0,C2] + x[Tr1,C1,C2] + x[Tr1,C3,C2]
   + x[Tr1,C4,C2] + x[Tr1,C5,C2] + x[Tr2,D0,C2] + x[Tr2,C1,C2]
   + x[Tr2,C3,C2] + x[Tr2,C4,C2] + x[Tr2,C5,C2] = 1
 Customer_C3_visited_once: x[Tr1,D0,C3] + x[Tr1,C1,C3] + x[Tr1,C2,C3]
   + x[Tr1,C4,C3] + x[Tr1,C5,C3] + x[Tr2,D0,C3] + x[Tr2,C1,C3]
   + x[Tr2,C2,C3] + x[Tr2,C4,C3] + x[Tr2,C5,C3] = 1
 Customer_C4_visited_once: x[Tr1,D0,C4] + x[Tr1,C1,C4] + x[Tr1,C2,C4]
   + x[Tr1,C3,C4] + x[Tr1,C5,C4] + x[Tr2,D0,C4] + x[Tr2,C1,C4]
   + x[Tr2,C2,C4] + x[Tr2,C3,C4] + x[Tr2,C5,C4] = 1
 Customer_C5_visited_once: x[Tr1,D0,C5] + x[Tr1,C1,C5] + x[Tr1,C2,C5]
   + x[Tr1,C3,C5] + x[Tr1,C4,C5] + x[Tr2,D0,C5] + x[Tr2,C1,C5]
   + x[Tr2,C2,C5] + x[Tr2,C3,C5] + x[Tr2,C4,C5] = 1
 Truck_leaves_depot_Tr1: x[Tr1,D0,C1] + x[Tr1,D0,C2] + x[Tr1,D0,C3]
   + x[Tr1,D0,C4] + x[Tr1,D0,C5] - y[Tr1] = 0
 Truck_leaves_depot_Tr2: x[Tr2,D0,C1] + x[Tr2,D0,C2] + x[Tr2,D0,C3]
   + x[Tr2,D0,C4] + x[Tr2,D0,C5] - y[Tr2] = 0
 Truck_returns_to_depot_Tr1: x[Tr1,C1,D0] + x[Tr1,C2,D0] + x[Tr1,C3,D0]
   + x[Tr1,C4,D0] + x[Tr1,C5,D0] - y[Tr1] = 0
 Truck_returns_to_depot_Tr2: x[Tr2,C1,D0] + x[Tr2,C2,D0] + x[Tr2,C3,D0]
   + x[Tr2,C4,D0] + x[Tr2,C5,D0] - y[Tr2] = 0
 Flow_balance_Tr1_C1: - x[Tr1,D0,C1] + x[Tr1,C1,D0] + x[Tr1,C1,C2]
   + x[Tr1,C1,C3] + x[Tr1,C1,C4] + x[Tr1,C1,C5] - x[Tr1,C2,C1]
   - x[Tr1,C3,C1] - x[Tr1,C4,C1] - x[Tr1,C5,C1] = 0
 Flow_balance_Tr1_C2: - x[Tr1,D0,C2] - x[Tr1,C1,C2] + x[Tr1,C2,D0]
   + x[Tr1,C2,C1] + x[Tr1,C2,C3] + x[Tr1,C2,C4] + x[Tr1,C2,C5]
   - x[Tr1,C3,C2] - x[Tr1,C4,C2] - x[Tr1,C5,C2] = 0
 Flow_balance_Tr1_C3: - x[Tr1,D0,C3] - x[Tr1,C1,C3] - x[Tr1,C2,C3]
   + x[Tr1,C3,D0] + x[Tr1,C3,C1] + x[Tr1,C3,C2] + x[Tr1,C3,C4]
   + x[Tr1,C3,C5] - x[Tr1,C4,C3] - x[Tr1,C5,C3] = 0
 Flow_balance_Tr1_C4: - x[Tr1,D0,C4] - x[Tr1,C1,C4] - x[Tr1,C2,C4]
   - x[Tr1,C3,C4] + x[Tr1,C4,D0] + x[Tr1,C4,C1] + x[Tr1,C4,C2]
   + x[Tr1,C4,C3] + x[Tr1,C4,C5] - x[Tr1,C5,C4] = 0
 Flow_balance_Tr1_C5: - x[Tr1,D0,C5] - x[Tr1,C1,C5] - x[Tr1,C2,C5]
   - x[Tr1,C3,C5] - x[Tr1,C4,C5] + x[Tr1,C5,D0] + x[Tr1,C5,C1]
   + x[Tr1,C5,C2] + x[Tr1,C5,C3] + x[Tr1,C5,C4] = 0
 Flow_balance_Tr2_C1: - x[Tr2,D0,C1] + x[Tr2,C1,D0] + x[Tr2,C1,C2]
   + x[Tr2,C1,C3] + x[Tr2,C1,C4] + x[Tr2,C1,C5] - x[Tr2,C2,C1]
   - x[Tr2,C3,C1] - x[Tr2,C4,C1] - x[Tr2,C5,C1] = 0
 Flow_balance_Tr2_C2: - x[Tr2,D0,C2] - x[Tr2,C1,C2] + x[Tr2,C2,D0]
   + x[Tr2,C2,C1] + x[Tr2,C2,C3] + x[Tr2,C2,C4] + x[Tr2,C2,C5]
   - x[Tr2,C3,C2] - x[Tr2,C4,C2] - x[Tr2,C5,C2] = 0
 Flow_balance_Tr2_C3: - x[Tr2,D0,C3] - x[Tr2,C1,C3] - x[Tr2,C2,C3]
   + x[Tr2,C3,D0] + x[Tr2,C3,C1] + x[Tr2,C3,C2] + x[Tr2,C3,C4]
   + x[Tr2,C3,C5] - x[Tr2,C4,C3] - x[Tr2,C5,C3] = 0
 Flow_balance_Tr2_C4: - x[Tr2,D0,C4] - x[Tr2,C1,C4] - x[Tr2,C2,C4]
   - x[Tr2,C3,C4] + x[Tr2,C4,D0] + x[Tr2,C4,C1] + x[Tr2,C4,C2]
   + x[Tr2,C4,C3] + x[Tr2,C4,C5] - x[Tr2,C5,C4] = 0
 Flow_balance_Tr2_C5: - x[Tr2,D0,C5] - x[Tr2,C1,C5] - x[Tr2,C2,C5]
   - x[Tr2,C3,C5] - x[Tr2,C4,C5] + x[Tr2,C5,D0] + x[Tr2,C5,C1]
   + x[Tr2,C5,C2] + x[Tr2,C5,C3] + x[Tr2,C5,C4] = 0
 Time_Tr1_D0_C1: - 6e+07 x[Tr1,D0,C1] - t[Tr1,D0] + t[Tr1,C1]
   >= -5.9999999641666666e+07
 Time_Tr1_D0_C2: - 6e+07 x[Tr1,D0,C2] - t[Tr1,D0] + t[Tr1,C2]
   >= -5.9999999563333333e+07
 Time_Tr1_D0_C3: - 6e+07 x[Tr1,D0,C3] - t[Tr1,D0] + t[Tr1,C3]
   >= -5.9999999475000001e+07
 Time_Tr1_D0_C4: - 6e+07 x[Tr1,D0,C4] - t[Tr1,D0] + t[Tr1,C4]
   >= -5.999999974666667e+07
 Time_Tr1_D0_C5: - 6e+07 x[Tr1,D0,C5] - t[Tr1,D0] + t[Tr1,C5]
   >= -5.9999999636666663e+07
 Time_Tr1_C1_D0: - 6e+07 x[Tr1,C1,D0] + t[Tr1,D0] - t[Tr1,C1]
   >= -5.9999999641666666e+07
 Time_Tr1_C1_C2: - 6e+07 x[Tr1,C1,C2] - t[Tr1,C1] + t[Tr1,C2]
   >= -5.9999999891666666e+07
 Time_Tr1_C1_C3: - 6e+07 x[Tr1,C1,C3] - t[Tr1,C1] + t[Tr1,C3]
   >= -5.9999999803333335e+07
 Time_Tr1_C1_C4: - 6e+07 x[Tr1,C1,C4] - t[Tr1,C1] + t[Tr1,C4]
   >= -5.9999999498333335e+07
 Time_Tr1_C1_C5: - 6e+07 x[Tr1,C1,C5] - t[Tr1,C1] + t[Tr1,C5]
   >= -5.9999999951666668e+07
 Time_Tr1_C2_D0: - 6e+07 x[Tr1,C2,D0] + t[Tr1,D0] - t[Tr1,C2]
   >= -5.9999999563333333e+07
 Time_Tr1_C2_C1: - 6e+07 x[Tr1,C2,C1] + t[Tr1,C1] - t[Tr1,C2]
   >= -5.9999999891666666e+07
 Time_Tr1_C2_C3: - 6e+07 x[Tr1,C2,C3] - t[Tr1,C2] + t[Tr1,C3]
   >= -5.9999999911666669e+07
 Time_Tr1_C2_C4: - 6e+07 x[Tr1,C2,C4] - t[Tr1,C2] + t[Tr1,C4]
   >= -5.9999999466666669e+07
 Time_Tr1_C2_C5: - 6e+07 x[Tr1,C2,C5] - t[Tr1,C2] + t[Tr1,C5]
   >= -5.9999999843333334e+07
 Time_Tr1_C3_D0: - 6e+07 x[Tr1,C3,D0] + t[Tr1,D0] - t[Tr1,C3]
   >= -5.9999999475000001e+07
 Time_Tr1_C3_C1: - 6e+07 x[Tr1,C3,C1] + t[Tr1,C1] - t[Tr1,C3]
   >= -5.9999999803333335e+07
 Time_Tr1_C3_C2: - 6e+07 x[Tr1,C3,C2] + t[Tr1,C2] - t[Tr1,C3]
   >= -5.9999999911666669e+07
 Time_Tr1_C3_C4: - 6e+07 x[Tr1,C3,C4] - t[Tr1,C3] + t[Tr1,C4]
   >= -5.9999999541666664e+07
 Time_Tr1_C3_C5: - 6e+07 x[Tr1,C3,C5] - t[Tr1,C3] + t[Tr1,C5]
   >= -5.9999999755000003e+07
 Time_Tr1_C4_D0: - 6e+07 x[Tr1,C4,D0] + t[Tr1,D0] - t[Tr1,C4]
   >= -5.999999974666667e+07
 Time_Tr1_C4_C1: - 6e+07 x[Tr1,C4,C1] + t[Tr1,C1] - t[Tr1,C4]
   >= -5.9999999498333335e+07
 Time_Tr1_C4_C2: - 6e+07 x[Tr1,C4,C2] + t[Tr1,C2] - t[Tr1,C4]
   >= -5.9999999466666669e+07
 Time_Tr1_C4_C3: - 6e+07 x[Tr1,C4,C3] + t[Tr1,C3] - t[Tr1,C4]
   >= -5.9999999541666664e+07
 Time_Tr1_C4_C5: - 6e+07 x[Tr1,C4,C5] - t[Tr1,C4] + t[Tr1,C5]
   >= -5.9999999493333332e+07
 Time_Tr1_C5_D0: - 6e+07 x[Tr1,C5,D0] + t[Tr1,D0] - t[Tr1,C5]
   >= -5.9999999636666663e+07
 Time_Tr1_C5_C1: - 6e+07 x[Tr1,C5,C1] + t[Tr1,C1] - t[Tr1,C5]
   >= -5.9999999951666668e+07
 Time_Tr1_C5_C2: - 6e+07 x[Tr1,C5,C2] + t[Tr1,C2] - t[Tr1,C5]
   >= -5.9999999843333334e+07
 Time_Tr1_C5_C3: - 6e+07 x[Tr1,C5,C3] + t[Tr1,C3] - t[Tr1,C5]
   >= -5.9999999755000003e+07
 Time_Tr1_C5_C4: - 6e+07 x[Tr1,C5,C4] + t[Tr1,C4] - t[Tr1,C5]
   >= -5.9999999493333332e+07
 Time_Tr2_D0_C1: - 6e+07 x[Tr2,D0,C1] - t[Tr2,D0] + t[Tr2,C1]
   >= -5.9999999641666666e+07
 Time_Tr2_D0_C2: - 6e+07 x[Tr2,D0,C2] - t[Tr2,D0] + t[Tr2,C2]
   >= -5.9999999563333333e+07
 Time_Tr2_D0_C3: - 6e+07 x[Tr2,D0,C3] - t[Tr2,D0] + t[Tr2,C3]
   >= -5.9999999475000001e+07
 Time_Tr2_D0_C4: - 6e+07 x[Tr2,D0,C4] - t[Tr2,D0] + t[Tr2,C4]
   >= -5.999999974666667e+07
 Time_Tr2_D0_C5: - 6e+07 x[Tr2,D0,C5] - t[Tr2,D0] + t[Tr2,C5]
   >= -5.9999999636666663e+07
 Time_Tr2_C1_D0: - 6e+07 x[Tr2,C1,D0] + t[Tr2,D0] - t[Tr2,C1]
   >= -5.9999999641666666e+07
 Time_Tr2_C1_C2: - 6e+07 x[Tr2,C1,C2] - t[Tr2,C1] + t[Tr2,C2]
   >= -5.9999999891666666e+07
 Time_Tr2_C1_C3: - 6e+07 x[Tr2,C1,C3] - t[Tr2,C1] + t[Tr2,C3]
   >= -5.9999999803333335e+07
 Time_Tr2_C1_C4: - 6e+07 x[Tr2,C1,C4] - t[Tr2,C1] + t[Tr2,C4]
   >= -5.9999999498333335e+07
 Time_Tr2_C1_C5: - 6e+07 x[Tr2,C1,C5] - t[Tr2,C1] + t[Tr2,C5]
   >= -5.9999999951666668e+07
 Time_Tr2_C2_D0: - 6e+07 x[Tr2,C2,D0] + t[Tr2,D0] - t[Tr2,C2]
   >= -5.9999999563333333e+07
 Time_Tr2_C2_C1: - 6e+07 x[Tr2,C2,C1] + t[Tr2,C1] - t[Tr2,C2]
   >= -5.9999999891666666e+07
 Time_Tr2_C2_C3: - 6e+07 x[Tr2,C2,C3] - t[Tr2,C2] + t[Tr2,C3]
   >= -5.9999999911666669e+07
 Time_Tr2_C2_C4: - 6e+07 x[Tr2,C2,C4] - t[Tr2,C2] + t[Tr2,C4]
   >= -5.9999999466666669e+07
 Time_Tr2_C2_C5: - 6e+07 x[Tr2,C2,C5] - t[Tr2,C2] + t[Tr2,C5]
   >= -5.9999999843333334e+07
 Time_Tr2_C3_D0: - 6e+07 x[Tr2,C3,D0] + t[Tr2,D0] - t[Tr2,C3]
   >= -5.9999999475000001e+07
 Time_Tr2_C3_C1: - 6e+07 x[Tr2,C3,C1] + t[Tr2,C1] - t[Tr2,C3]
   >= -5.9999999803333335e+07
 Time_Tr2_C3_C2: - 6e+07 x[Tr2,C3,C2] + t[Tr2,C2] - t[Tr2,C3]
   >= -5.9999999911666669e+07
 Time_Tr2_C3_C4: - 6e+07 x[Tr2,C3,C4] - t[Tr2,C3] + t[Tr2,C4]
   >= -5.9999999541666664e+07
 Time_Tr2_C3_C5: - 6e+07 x[Tr2,C3,C5] - t[Tr2,C3] + t[Tr2,C5]
   >= -5.9999999755000003e+07
 Time_Tr2_C4_D0: - 6e+07 x[Tr2,C4,D0] + t[Tr2,D0] - t[Tr2,C4]
   >= -5.999999974666667e+07
 Time_Tr2_C4_C1: - 6e+07 x[Tr2,C4,C1] + t[Tr2,C1] - t[Tr2,C4]
   >= -5.9999999498333335e+07
 Time_Tr2_C4_C2: - 6e+07 x[Tr2,C4,C2] + t[Tr2,C2] - t[Tr2,C4]
   >= -5.9999999466666669e+07
 Time_Tr2_C4_C3: - 6e+07 x[Tr2,C4,C3] + t[Tr2,C3] - t[Tr2,C4]
   >= -5.9999999541666664e+07
 Time_Tr2_C4_C5: - 6e+07 x[Tr2,C4,C5] - t[Tr2,C4] + t[Tr2,C5]
   >= -5.9999999493333332e+07
 Time_Tr2_C5_D0: - 6e+07 x[Tr2,C5,D0] + t[Tr2,D0] - t[Tr2,C5]
   >= -5.9999999636666663e+07
 Time_Tr2_C5_C1: - 6e+07 x[Tr2,C5,C1] + t[Tr2,C1] - t[Tr2,C5]
   >= -5.9999999951666668e+07
 Time_Tr2_C5_C2: - 6e+07 x[Tr2,C5,C2] + t[Tr2,C2] - t[Tr2,C5]
   >= -5.9999999843333334e+07
 Time_Tr2_C5_C3: - 6e+07 x[Tr2,C5,C3] + t[Tr2,C3] - t[Tr2,C5]
   >= -5.9999999755000003e+07
 Time_Tr2_C5_C4: - 6e+07 x[Tr2,C5,C4] + t[Tr2,C4] - t[Tr2,C5]
   >= -5.9999999493333332e+07
 Payload_Tr1: 3.85 x[Tr1,C1,D0] + 3.85 x[Tr1,C1,C2] + 3.85 x[Tr1,C1,C3]
   + 3.85 x[Tr1,C1,C4] + 3.85 x[Tr1,C1,C5] + 4.89 x[Tr1,C2,D0]
   + 4.89 x[Tr1,C2,C1] + 4.89 x[Tr1,C2,C3] + 4.89 x[Tr1,C2,C4]
   + 4.89 x[Tr1,C2,C5] + 1.4 x[Tr1,C3,D0] + 1.4 x[Tr1,C3,C1]
   + 1.4 x[Tr1,C3,C2] + 1.4 x[Tr1,C3,C4] + 1.4 x[Tr1,C3,C5]
   + 4.69 x[Tr1,C4,D0] + 4.69 x[Tr1,C4,C1] + 4.69 x[Tr1,C4,C2]
   + 4.69 x[Tr1,C4,C3] + 4.69 x[Tr1,C4,C5] + 3.51 x[Tr1,C5,D0]
   + 3.51 x[Tr1,C5,C1] + 3.51 x[Tr1,C5,C2] + 3.51 x[Tr1,C5,C3]
   + 3.51 x[Tr1,C5,C4] <= 1000
 Payload_Tr2: 3.85 x[Tr2,C1,D0] + 3.85 x[Tr2,C1,C2] + 3.85 x[Tr2,C1,C3]
   + 3.85 x[Tr2,C1,C4] + 3.85 x[Tr2,C1,C5] + 4.89 x[Tr2,C2,D0]
   + 4.89 x[Tr2,C2,C1] + 4.89 x[Tr2,C2,C3] + 4.89 x[Tr2,C2,C4]
   + 4.89 x[Tr2,C2,C5] + 1.4 x[Tr2,C3,D0] + 1.4 x[Tr2,C3,C1]
   + 1.4 x[Tr2,C3,C2] + 1.4 x[Tr2,C3,C4] + 1.4 x[Tr2,C3,C5]
   + 4.69 x[Tr2,C4,D0] + 4.69 x[Tr2,C4,C1] + 4.69 x[Tr2,C4,C2]
   + 4.69 x[Tr2,C4,C3] + 4.69 x[Tr2,C4,C5] + 3.51 x[Tr2,C5,D0]
   + 3.51 x[Tr2,C5,C1] + 3.51 x[Tr2,C5,C2] + 3.51 x[Tr2,C5,C3]
   + 3.51 x[Tr2,C5,C4] <= 1000
 Link_yTr1_to_x_Tr1: - x[Tr1,D0,C1] - x[Tr1,D0,C2] - x[Tr1,D0,C3]
   - x[Tr1,D0,C4] - x[Tr1,D0,C5] + y[Tr1] = 0
 Link_yTr2_to_x_Tr2: - x[Tr2,D0,C1] - x[Tr2,D0,C2] - x[Tr2,D0,C3]
   - x[Tr2,D0,C4] - x[Tr2,D0,C5] + y[Tr2] = 0
 Update_max_delivery_time_Tr1_C1: - t[Tr1,C1] + t_max >= 0
 Update_max_delivery_time_Tr1_C2: - t[Tr1,C2] + t_max >= 0
 Update_max_delivery_time_Tr1_C3: - t[Tr1,C3] + t_max >= 0
 Update_max_delivery_time_Tr1_C4: - t[Tr1,C4] + t_max >= 0
 Update_max_delivery_time_Tr1_C5: - t[Tr1,C5] + t_max >= 0
 Update_max_delivery_time_Tr2_C1: - t[Tr2,C1] + t_max >= 0
 Update_max_delivery_time_Tr2_C2: - t[Tr2,C2] + t_max >= 0
 Update_max_delivery_time_Tr2_C3: - t[Tr2,C3] + t_max >= 0
 Update_max_delivery_time_Tr2_C4: - t[Tr2,C4] + t_max >= 0
 Update_max_delivery_time_Tr2_C5: - t[Tr2,C5] + t_max >= 0
 Update_time_Tr1_C1: - 0.3583333333333333 x[Tr1,D0,C1]
   - 0.1083333333333334 x[Tr1,C2,C1] - 0.1966666666666667 x[Tr1,C3,C1]
   - 0.5016666666666666 x[Tr1,C4,C1] - 0.0483333333333333 x[Tr1,C5,C1]
   + t[Tr1,C1] + [ - x[Tr1,D0,C1] * t[Tr1,D0] - x[Tr1,C2,C1] * t[Tr1,C2]
   - x[Tr1,C3,C1] * t[Tr1,C3] - x[Tr1,C4,C1] * t[Tr1,C4]
   - x[Tr1,C5,C1] * t[Tr1,C5] ] = 0
 Update_time_Tr1_C2: - 0.4366666666666667 x[Tr1,D0,C2]
   - 0.1083333333333334 x[Tr1,C1,C2] - 0.0883333333333333 x[Tr1,C3,C2]
   - 0.5333333333333333 x[Tr1,C4,C2] - 0.1566666666666667 x[Tr1,C5,C2]
   + t[Tr1,C2] + [ - x[Tr1,D0,C2] * t[Tr1,D0] - x[Tr1,C1,C2] * t[Tr1,C1]
   - x[Tr1,C3,C2] * t[Tr1,C3] - x[Tr1,C4,C2] * t[Tr1,C4]
   - x[Tr1,C5,C2] * t[Tr1,C5] ] = 0
 Update_time_Tr1_C3: - 0.525 x[Tr1,D0,C3] - 0.1966666666666667 x[Tr1,C1,C3]
   - 0.0883333333333333 x[Tr1,C2,C3] - 0.4583333333333333 x[Tr1,C4,C3]
   - 0.245 x[Tr1,C5,C3] + t[Tr1,C3] + [ - x[Tr1,D0,C3] * t[Tr1,D0]
   - x[Tr1,C1,C3] * t[Tr1,C1] - x[Tr1,C2,C3] * t[Tr1,C2]
   - x[Tr1,C4,C3] * t[Tr1,C4] - x[Tr1,C5,C3] * t[Tr1,C5] ] = 0
 Update_time_Tr1_C4: - 0.2533333333333334 x[Tr1,D0,C4]
   - 0.5016666666666666 x[Tr1,C1,C4] - 0.5333333333333333 x[Tr1,C2,C4]
   - 0.4583333333333333 x[Tr1,C3,C4] - 0.5066666666666666 x[Tr1,C5,C4]
   + t[Tr1,C4] + [ - x[Tr1,D0,C4] * t[Tr1,D0] - x[Tr1,C1,C4] * t[Tr1,C1]
   - x[Tr1,C2,C4] * t[Tr1,C2] - x[Tr1,C3,C4] * t[Tr1,C3]
   - x[Tr1,C5,C4] * t[Tr1,C5] ] = 0
 Update_time_Tr1_C5: - 0.3633333333333333 x[Tr1,D0,C5]
   - 0.0483333333333333 x[Tr1,C1,C5] - 0.1566666666666667 x[Tr1,C2,C5]
   - 0.245 x[Tr1,C3,C5] - 0.5066666666666666 x[Tr1,C4,C5] + t[Tr1,C5] + [
   - x[Tr1,D0,C5] * t[Tr1,D0] - x[Tr1,C1,C5] * t[Tr1,C1]
   - x[Tr1,C2,C5] * t[Tr1,C2] - x[Tr1,C3,C5] * t[Tr1,C3]
   - x[Tr1,C4,C5] * t[Tr1,C4] ] = 0
 Update_time_Tr2_C1: - 0.3583333333333333 x[Tr2,D0,C1]
   - 0.1083333333333334 x[Tr2,C2,C1] - 0.1966666666666667 x[Tr2,C3,C1]
   - 0.5016666666666666 x[Tr2,C4,C1] - 0.0483333333333333 x[Tr2,C5,C1]
   + t[Tr2,C1] + [ - x[Tr2,D0,C1] * t[Tr2,D0] - x[Tr2,C2,C1] * t[Tr2,C2]
   - x[Tr2,C3,C1] * t[Tr2,C3] - x[Tr2,C4,C1] * t[Tr2,C4]
   - x[Tr2,C5,C1] * t[Tr2,C5] ] = 0
 Update_time_Tr2_C2: - 0.4366666666666667 x[Tr2,D0,C2]
   - 0.1083333333333334 x[Tr2,C1,C2] - 0.0883333333333333 x[Tr2,C3,C2]
   - 0.5333333333333333 x[Tr2,C4,C2] - 0.1566666666666667 x[Tr2,C5,C2]
   + t[Tr2,C2] + [ - x[Tr2,D0,C2] * t[Tr2,D0] - x[Tr2,C1,C2] * t[Tr2,C1]
   - x[Tr2,C3,C2] * t[Tr2,C3] - x[Tr2,C4,C2] * t[Tr2,C4]
   - x[Tr2,C5,C2] * t[Tr2,C5] ] = 0
 Update_time_Tr2_C3: - 0.525 x[Tr2,D0,C3] - 0.1966666666666667 x[Tr2,C1,C3]
   - 0.0883333333333333 x[Tr2,C2,C3] - 0.4583333333333333 x[Tr2,C4,C3]
   - 0.245 x[Tr2,C5,C3] + t[Tr2,C3] + [ - x[Tr2,D0,C3] * t[Tr2,D0]
   - x[Tr2,C1,C3] * t[Tr2,C1] - x[Tr2,C2,C3] * t[Tr2,C2]
   - x[Tr2,C4,C3] * t[Tr2,C4] - x[Tr2,C5,C3] * t[Tr2,C5] ] = 0
 Update_time_Tr2_C4: - 0.2533333333333334 x[Tr2,D0,C4]
   - 0.5016666666666666 x[Tr2,C1,C4] - 0.5333333333333333 x[Tr2,C2,C4]
   - 0.4583333333333333 x[Tr2,C3,C4] - 0.5066666666666666 x[Tr2,C5,C4]
   + t[Tr2,C4] + [ - x[Tr2,D0,C4] * t[Tr2,D0] - x[Tr2,C1,C4] * t[Tr2,C1]
   - x[Tr2,C2,C4] * t[Tr2,C2] - x[Tr2,C3,C4] * t[Tr2,C3]
   - x[Tr2,C5,C4] * t[Tr2,C5] ] = 0
 Update_time_Tr2_C5: - 0.3633333333333333 x[Tr2,D0,C5]
   - 0.0483333333333333 x[Tr2,C1,C5] - 0.1566666666666667 x[Tr2,C2,C5]
   - 0.245 x[Tr2,C3,C5] - 0.5066666666666666 x[Tr2,C4,C5] + t[Tr2,C5] + [
   - x[Tr2,D0,C5] * t[Tr2,D0] - x[Tr2,C1,C5] * t[Tr2,C1]
   - x[Tr2,C2,C5] * t[Tr2,C2] - x[Tr2,C3,C5] * t[Tr2,C3]
   - x[Tr2,C4,C5] * t[Tr2,C4] ] = 0
Bounds
Binaries
 x[Tr1,D0,C1] x[Tr1,D0,C2] x[Tr1,D0,C3] x[Tr1,D0,C4] x[Tr1,D0,C5]
 x[Tr1,C1,D0] x[Tr1,C1,C2] x[Tr1,C1,C3] x[Tr1,C1,C4] x[Tr1,C1,C5]
 x[Tr1,C2,D0] x[Tr1,C2,C1] x[Tr1,C2,C3] x[Tr1,C2,C4] x[Tr1,C2,C5]
 x[Tr1,C3,D0] x[Tr1,C3,C1] x[Tr1,C3,C2] x[Tr1,C3,C4] x[Tr1,C3,C5]
 x[Tr1,C4,D0] x[Tr1,C4,C1] x[Tr1,C4,C2] x[Tr1,C4,C3] x[Tr1,C4,C5]
 x[Tr1,C5,D0] x[Tr1,C5,C1] x[Tr1,C5,C2] x[Tr1,C5,C3] x[Tr1,C5,C4]
 x[Tr2,D0,C1] x[Tr2,D0,C2] x[Tr2,D0,C3] x[Tr2,D0,C4] x[Tr2,D0,C5]
 x[Tr2,C1,D0] x[Tr2,C1,C2] x[Tr2,C1,C3] x[Tr2,C1,C4] x[Tr2,C1,C5]
 x[Tr2,C2,D0] x[Tr2,C2,C1] x[Tr2,C2,C3] x[Tr2,C2,C4] x[Tr2,C2,C5]
 x[Tr2,C3,D0] x[Tr2,C3,C1] x[Tr2,C3,C2] x[Tr2,C3,C4] x[Tr2,C3,C5]
 x[Tr2,C4,D0] x[Tr2,C4,C1] x[Tr2,C4,C2] x[Tr2,C4,C3] x[Tr2,C4,C5]
 x[Tr2,C5,D0] x[Tr2,C5,C1] x[Tr2,C5,C2] x[Tr2,C5,C3] x[Tr2,C5,C4] y[Tr1]
 y[Tr2]
End
