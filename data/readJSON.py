import json
from datetime import datetime
import xlsxwriter

fileDir = 'download_weather-station.json'

with open(fileDir, 'r') as data_file:
    data = json.load(data_file)


for element in list(data):
    if element != 'variables':
        data.pop(element, None)

workbook = xlsxwriter.Workbook('Dados_Estacao_LoRaWAN.xlsx')
worksheet = workbook.add_worksheet()

worksheet.write(1, 0, "Data/Hora")
col = 1

with open('data.txt', 'w') as data_file:
    for variable in list(data['variables']):
        #print('\n')
        #print('Variavel: ', variable['variable_name'])
        #data_file.write("\n\nVariavel: ")
        #data_file.write(variable['variable_name'])
        #data_file.write("\n")
        row = 1
        worksheet.write(row, col, variable['variable_name'])
        row += 1
        col += 1
        
        for v in variable['histories']:
            #print('Value:', v['value'])
            v['timestamp'] = datetime.utcfromtimestamp(v['timestamp']/1000).strftime('%Y-%m-%d %H:%M')
            #print('Timestamp:', v['timestamp'])

            #data_file.write("Valor: "); data_file.write(v['value']); data_file.write("\n")
            #data_file.write("Timestamp: "); data_file.write(str(v['timestamp'])); data_file.write("\n")
            worksheet.write(row, 0, str(v['timestamp']))
            worksheet.write(row, col-1, v['value'])
            row += 1


print("Done")
workbook.close()