import re
f_in = open('0526gps_data')
f_out = open('20200526gps_data','w')
a=f_in.readlines()
for line in a[1:]:
	m=re.match(r'^(-?\w*\,){6}([\d\-]*\.\d*)\,([\d\-]*\.\d*)\,([\d\-]*\.\d*)\,(\w*\.?\w*\,){9}(\d*)$',line)
	lat=m.group(2)
	#lat=lat.replace('.','')	
	lon=m.group(3)
        #lon=lon.replace('.','')	
	gpsData = lat+' '+lon+'\n'
	f_out.write(gpsData)
f_in.close()
f_out.close()

