# import xlrd
import time
import csv

class Interface:
    def __init__(self, params_file_path, range_file_path):
        self.params_file_path = params_file_path
        self.range_file_path = range_file_path
        self.index_lst=[]
        self.param_dict={"ego_velocity":[], "target_velocity":[], "ttc":[], "radius":[], "friction_coeeficient":[], "lane_width":[], "motor_separamstion_infrastructure":[], "none_motor_lane_nr":[], "central_separamstion_infrastructure":[], "lane_line_type":[], "lane_nr":[], "same_direction_lane_nr":[], "lane_index":[], "weather":[]}
        with open(self.params_file_path, newline='') as param_csvfile:
            # reader = csv.DictReader(csvfile)
            self.param_reader = csv.reader(param_csvfile)
            for row in self.param_reader:
                self.param_dict["ego_velocity"].append(row[2])
                self.param_dict["target_velocity"].append(row[3])
                self.param_dict["ttc"].append(row[4])
                self.param_dict["radius"].append(row[5])
                self.param_dict["friction_coeeficient"].append(row[6])
                self.param_dict["lane_width"].append(row[7])
                self.param_dict["motor_separamstion_infrastructure"].append(row[8])
                self.param_dict["none_motor_lane_nr"].append(row[9])
                self.param_dict["central_separamstion_infrastructure"].append(row[10])
                self.param_dict["lane_line_type"].append(row[11])
                self.param_dict["lane_nr"].append(row[12])
                self.param_dict["same_direction_lane_nr"].append(row[13])
                self.param_dict["lane_index"].append(row[14])
                self.param_dict["weather"].append(row[15])
        # print("param_dict:", self.param_dict)    
        self.ego_velocity_range = []
        self.target_velocity_range = []
        self.ttc_range = []
        self.radius_range = []
        self.friction_coeeficient_range = []
        self.lane_width_range = []
        self.motor_separamstion_infrastructure_range = []
        self.none_motor_lane_nr_range = []
        self.central_separamstion_infrastructure_range = []
        self.lane_line_type_range = []
        self.lane_nr_range = []
        self.same_direction_lane_nr_range = []
        self.lane_index_range = []
        self.weather_range = []
        with open(self.range_file_path, newline='') as range_csvfile:
            # reader = csv.DictReader(csvfile)
            self.range_reader = csv.reader(range_csvfile)
            for row in self.range_reader:
                self.ego_velocity_range.append(row[2])
                self.target_velocity_range.append(row[3])
                self.ttc_range.append(row[4])
                self.radius_range.append(row[5])
                self.friction_coeeficient_range.append(row[6])
                self.lane_width_range.append(row[7])
                self.motor_separamstion_infrastructure_range.append(row[8])
                self.none_motor_lane_nr_range.append(row[9])
                self.central_separamstion_infrastructure_range.append(row[10])
                self.lane_line_type_range.append(row[11])
                self.lane_nr_range.append(row[12])
                self.same_direction_lane_nr_range.append(row[13])
                self.lane_index_range.append(row[14])
                self.weather_range.append(row[15])



        # self.params_var_sheet = csv.DictReader(self.params_file_path)
        # self.range_sheet = csv.DictReader(self.range_file_path)
        # self.nrows = self.params_var_sheet.nrows - 1
        self.current_test_id=0
        self.invalid_dict={"ego_velocity":[], "target_velocity":[], "ttc":[], "radius":[], "friction_coeeficient":[], "lane_width":[], "motor_separamstion_infrastructure":[], "none_motor_lane_nr":[], "central_separamstion_infrastructure":[], "lane_line_type":[], "lane_nr":[], "same_direction_lane_nr":[], "lane_index":[], "weather":[]}
        
    
    def next(self):
        self.current_test_id += 1
        self.ego_init_speed = float(self.param_dict["ego_velocity"][self.current_test_id])
        self.target_init_speed = float(self.param_dict["target_velocity"][self.current_test_id])
        self.ttc = float(self.param_dict["ttc"][self.current_test_id])
        self.radius = float(self.param_dict["radius"][self.current_test_id])
        self.friction_coeeficient = float(self.param_dict["friction_coeeficient"][self.current_test_id])
        self.lane_width = float(self.param_dict["lane_width"][self.current_test_id])
        self.motor_separamstion_infrastructure = self.param_dict["motor_separamstion_infrastructure"][self.current_test_id]
        self.none_motor_lane_nr = self.param_dict["none_motor_lane_nr"][self.current_test_id]
        self.central_separamstion_infrastructure = self.param_dict["central_separamstion_infrastructure"][self.current_test_id]
        self.lane_line_type = self.param_dict["lane_line_type"][self.current_test_id]
        self.lane_nr = int(self.param_dict["lane_nr"][self.current_test_id])
        self.same_direction_lane_nr = int(self.param_dict["same_direction_lane_nr"][self.current_test_id])
        self.lane_index = int(self.param_dict["lane_index"][self.current_test_id])
        self.weather = self.param_dict["weather"][self.current_test_id]
        return self

    def check_valid(self):
        # check the sanity of the all test parameters and return to the invalid parameters index dictionary
        # should be done before the test run
        with open(self.params_file_path, newline='') as param_csvfile:
            # reader = csv.DictReader(csvfile)
            self.param_reader = csv.reader(param_csvfile)
            for index, row in enumerate(self.param_reader):
                if index != 0:
                    self.index_lst.append(index)
                    if float(row[2]) > float(self.ego_velocity_range[2]) or float(row[2]) < float(self.ego_velocity_range[1]):
                        self.invalid_dict["ego_velocity"].append(row[1])
                    if float(row[3]) > float(self.target_velocity_range[2]) or float(row[3]) < float(self.target_velocity_range[1]):
                        self.invalid_dict["target_velocity"].append(row[1])
                    if float(row[4]) > float(self.ttc_range[2]) or float(row[4]) < float(self.ttc_range[1]):
                        self.invalid_dict["ttc"].append(row[1])
                    if float(row[5]) > float(self.radius_range[2]) or float(row[5]) < float(self.radius_range[1]):
                        self.invalid_dict["radius"].append(row[1])
                    if float(row[6]) > float(self.friction_coeeficient_range[2]) or float(row[6]) < float(self.friction_coeeficient_range[1]):
                        self.invalid_dict["friction_coeeficient"].append(row[1])
                    if float(row[7]) > float(self.lane_width_range[2]) or float(row[7])< float(self.lane_width_range[1]):
                        self.invalid_dict["lane_width"].append(row[1])
                    if row[8] not in self.motor_separamstion_infrastructure_range:
                        self.invalid_dict["motor_separamstion_infrastructure"].append(row[1])
                    if row[9] not in self.none_motor_lane_nr_range:
                        self.invalid_dict["none_motor_lane_nr"].append(row[1])
                    if row[10] not in self.central_separamstion_infrastructure_range:
                        self.invalid_dict["central_separamstion_infrastructure"].append(row[1])
                    if row[11] not in self.lane_line_type_range:
                        self.invalid_dict["lane_line_type"].append(row[1])
                    if float(row[12]) > float(self.lane_nr_range[2]) or float(row[12]) < float(self.lane_nr_range[1]) or float(row[12].isdigit()) != True:
                        self.invalid_dict["lane_nr"].append(row[1])
                    if float(row[13]) > float(self.same_direction_lane_nr_range[2]) or float(row[13]) < float(self.same_direction_lane_nr_range[1]) or row[13].isdigit() != True:
                        self.invalid_dict["same_direction_lane_nr"].append(row[1])
                    if float(row[14]) > float(row[13]) or float(row[14]) < 1 or row[14].isdigit() != True:
                        self.invalid_dict["lane_index"].append(row[1])
                    if row[15] not in self.weather_range:
                        self.invalid_dict["weather"].append(row[1])
        
        return self.invalid_dict


if __name__ == '__main__':
    # init the interface by passing the file path
    # scenario_params = Interface("/home/minerva/WorkSpace/Minerva/simulation/digital-testing-product/drivesim_ov/ds2_scripts/test_case_parameter_list.xlsx")
    # scenario_params = Interface("/drivesim-ov/drivesim2_scripts/test_case_parameter_list.csv", "/drivesim-ov/drivesim2_scripts/valid_range.csv")
    scenario_params = Interface("/home/minerva/WorkSpace/Minerva/simulation/digital-testing-product/drivesim_ov/ds2_scripts/test_case_parameter_list.csv", "/home/minerva/WorkSpace/Minerva/simulation/digital-testing-product/drivesim_ov/ds2_scripts/valid_range.csv")
    
    # check the sanity of all the test parameters before test run
    invalid_params_dict = scenario_params.check_valid()
    # get the dictionary of invalid parameters
    print("invalid parameters:", invalid_params_dict)

    current_params=scenario_params.next()

    print("current_params:",current_params)
    current_ego_velocity = current_params.ego_init_speed
    print("current_ego_velocity", current_ego_velocity)
    print("test case numbers:",len(current_params.index_lst))
    # set the progress bar
    '''
    # experiment without valid check break
    for i in pbar:
            time.sleep(0.05)
            # update the test parameters
            current_params = scenario_params.next()
            print("----------------------------test %d parameters---------------------------"%current_params.current_test_id)
            pbar.set_description('Processing test '+str(current_params.current_test_id))
            # print(current_params.range_sheet.col_values(2))
            print("current row of parameters:", current_params.current_params_row)
            print("current velocity:",current_params.ego_velocity)
    '''
    # if any of the test parameters is invalid, then the test won't start
    # while invalid_params_dict.values():
    #     print("invalid parameters detected!")
    #     break
    # # test loop start when all test parameters pass the sanity test
    # else:

    # current_params = scenario_params.next()
    # print("----------------------------test %d parameters---------------------------"%current_params.current_test_id)
    # print(current_params.range_sheet.col_values(2))
    # print("current row of parameters:", current_params.current_params_row)
    # print("current velocity:",current_params.ego_velocity)

