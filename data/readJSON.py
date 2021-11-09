import json
from datetime import datetime 

fileDir = 'data\WeatherStationData_09-11.json'

with open(fileDir, 'r') as data_file:
    data = json.load(data_file)


for element in list(data):
    if element != 'variables':
        data.pop(element, None)


with open('data\data.txt', 'w') as data_file:
    for variable in list(data['variables']):
        print('\n')
        print('Variavel: ', variable['variable_name'])
        data_file.write("\n\nVariavel: ")
        data_file.write(variable['variable_name'])
        data_file.write("\n")

        for v in variable['histories']:
            print('Value:', v['value'])
            v['timestamp'] = datetime.fromtimestamp(v['timestamp']/1000)
            print('Timestamp:', v['timestamp'])

            data_file.write("Valor: "); data_file.write(v['value']); data_file.write("\n")
            data_file.write("Timestamp: "); data_file.write(str(v['timestamp'])); data_file.write("\n")