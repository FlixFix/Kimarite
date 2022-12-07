import Paths as params
import Sumo.SumoHelperFcts as shf


network_settings = params.NETWORK("ValidationNetwork.net.xml")
paths = params.SUMO_PATHS()

SCENARIO_PATH = params.SUMO_PATHS().scenarios

used_scenario = "1a"
test_scenario = shf.Scenario(used_scenario, network_settings.net)
test_scenario.import_scenario()
_, dataset = test_scenario.get_average_trajectory(add_header=False)
# save to csv file
dataset.to_csv(SCENARIO_PATH +  r'\MyScenarios\Scenario2\tum_data.csv',
               index=False, header=True)
dataset.to_csv(SCENARIO_PATH + r'\MyScenarios\Scenario3\tum_data.csv',
               index=False, header=True)


used_scenario = "1d"
test_scenario = shf.Scenario(used_scenario, network_settings.net)
test_scenario.import_scenario()
_, dataset = test_scenario.get_average_trajectory(add_header=False)
# save to csv file
dataset.to_csv(SCENARIO_PATH + r'\MyScenarios\Scenario1\tum_data.csv',
               index=False, header=True)


used_scenario = "1e"
test_scenario = shf.Scenario(used_scenario, network_settings.net)
test_scenario.import_scenario()
_, dataset = test_scenario.get_average_trajectory(add_header=False)
# save to csv file
dataset.to_csv(SCENARIO_PATH + r'\MyScenarios\Scenario5\tum_data.csv',
               index=False, header=True)


used_scenario = "2a"
test_scenario = shf.Scenario(used_scenario, network_settings.net)
test_scenario.import_scenario()
_, dataset = test_scenario.get_average_trajectory(add_header=False)
# save to csv file
dataset.to_csv(SCENARIO_PATH + r'\MyScenarios\Scenario4\tum_data.csv',
               index=False, header=True)

print("Average Scenario Data Created and Saved!")