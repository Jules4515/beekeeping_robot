import yaml
import time

input_yaml = '../config/waypoints_GPS.yaml'
output_kml = f'trajectoire_reelle_{int(time.time())}.kml'

# Chargement du fichier YAML
with open(input_yaml, 'r') as yml_file:
    data = yaml.safe_load(yml_file)

coordinates_list = []

# Extraction des coordonnées (Le format KML exige: longitude,latitude,altitude)
if 'waypoints_GPS' in data:
    for wp in data['waypoints_GPS']:
        lat = wp.get('latitude', 0.0)
        lon = wp.get('longitude', 0.0)
        alt = 0.0  # Altitude forcée au sol
        
        coordinates_list.append(f"{lon},{lat},{alt}")

# Formatage des coordonnées avec sauts de ligne pour respecter le standard XML
coordinates_str = "\n                ".join(coordinates_list)

# Construction du template KML (LineString continu)
kml_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Trajectoire Robot GPS</name>
    <description>Généré automatiquement depuis {input_yaml}</description>
    
    <!-- Définition du style visuel de la ligne (Rouge, épaisseur 3) -->
    <Style id="redLine">
      <LineStyle>
        <color>ff0000ff</color> <!-- Format AABBGGRR : opacité=ff, bleu=00, vert=00, rouge=ff -->
        <width>3</width>
      </LineStyle>
    </Style>
    
    <Placemark>
      <name>Path MPPI</name>
      <styleUrl>#redLine</styleUrl>
      <LineString>
        <tessellate>1</tessellate> <!-- Plaque la ligne sur le relief du terrain -->
        <coordinates>
                {coordinates_str}
        </coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>
"""

# Écriture du fichier KML
with open(output_kml, 'w') as kml_file:
    kml_file.write(kml_content)

print(f"Conversion terminée : {len(coordinates_list)} points exportés vers {output_kml}.")