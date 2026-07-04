import yaml
import csv
import time

input_yaml = '../config/waypoints_GPS.yaml'
output_csv = f'trajectoire_reelle_{int(time.time())}.csv'

# Chargement du fichier YAML
with open(input_yaml, 'r') as yml_file:
    data = yaml.safe_load(yml_file)

# Création du fichier CSV
with open(output_csv, 'w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    
    # Entêtes exigés par ta configuration QGIS
    csv_writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude'])
    
    # Extraction et écriture
    if 'waypoints_GPS' in data:
        for idx, wp in enumerate(data['waypoints_GPS']):
            timestamp = round(idx * 0.1, 1)  # Incrément strict de 100ms
            lat = wp.get('latitude', 0.0)
            lon = wp.get('longitude', 0.0)
            alt = 0.0  # Altitude factice par défaut
            
            csv_writer.writerow([timestamp, lat, lon, alt])

print(f"Conversion terminée : {len(data['waypoints_GPS'])} points exportés vers {output_csv}.")