import zipfile
import os
import json

def export_to_dji_kmz(waypoints, output_path="mission_dji.kmz"):
    print(f"[Exporter] Запис DJI WPML файлу -> {output_path}")
    kml_content = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.3">
  <Document>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:takeOffSecurityHeight>15</wpml:takeOffSecurityHeight>
      <wpml:globalTransitionalSpeed>5</wpml:globalTransitionalSpeed>
    </wpml:missionConfig>
    <Folder>
      <wpml:templateId>0</wpml:templateId>
      <wpml:autoFlightSpeed>4</wpml:autoFlightSpeed>
      <wpml:executeHeightMode>relativeToStartPoint</wpml:executeHeightMode>
"""

    for i, (lat, lon, alt, pitch) in enumerate(waypoints):
        kml_content += f"""      <Placemark>
        <Point>
          <coordinates>{lon},{lat}</coordinates>
        </Point>
        <wpml:index>{i}</wpml:index>
        <wpml:executeHeight>{alt}</wpml:executeHeight>
        <wpml:gimbalPitchAngle>{pitch}</wpml:gimbalPitchAngle>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
        </wpml:waypointHeadingParam>
      </Placemark>
"""

    kml_content += """    </Folder>
  </Document>
</kml>
"""

    temp_filename = "template.kml"
    with open(temp_filename, "w", encoding="utf-8") as f:
        f.write(kml_content)

    with zipfile.ZipFile(output_path, 'w', zipfile.ZIP_DEFLATED) as kmz:
        kmz.write(temp_filename, "template.kml")

    os.remove(temp_filename)

def export_to_qgc_plan(waypoints, output_path="mission_qgc.plan"):
    print(f"[Exporter] Запис QGroundControl JSON файлу -> {output_path}")
    if not waypoints:
        return

    items = []
    home_lat, home_lon, _, _ = waypoints[0]

    for i, (lat, lon, alt, pitch) in enumerate(waypoints):
        # Навігаційний вейпоінт
        items.append({
            "autoContinue": True,
            "command": 16,
            "frame": 3,
            "params": [0, 0, 0, None, lat, lon, alt],
            "type": "SimpleItem"
        })
        # Керування нахилом камери
        items.append({
            "autoContinue": True,
            "command": 205,
            "frame": 2,
            "params": [pitch, 0, 0, None, None, None, 2],
            "type": "SimpleItem"
        })

    plan_data = {
        "fileType": "Plan",
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": 5,
            "hoverSpeed": 3,
            "firmwareType": 12,
            "vehicleType": 2,
            "version": 2,
            "plannedHomePosition": [home_lat, home_lon, 0],
            "items": items
        },
        "version": 1
    }

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(plan_data, f, indent=2)