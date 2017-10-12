import utm

from waternet_vertex import WaternetVertex

# Convert GPS string to UTM
def convert_GPS_json_to_UTM_position(position_string):
	utm_coordinate = utm.from_latlon(float(position_string["geo_lat"]), float(position_string["geo_lng"]))[:2] # only use the first two elements of the tuple: easting and northing (in that order)
	
	return WaternetVertex(utm_coordinate[0], utm_coordinate[1])

