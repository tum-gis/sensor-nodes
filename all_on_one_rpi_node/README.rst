All-on-one Rpi Node
===================

This sensor node is made to showcase a use-case of Raspberry Pi for 
a complete all on one sensor node. For achieving this a DHT-22 sensor 
along with a digital light sensor was used to measure temperature, 
humidity, and light. The sensor readings were directly pushed to the
FROST Server running on the Pi itself. As a result, the Pi act as an 
independent sensor system running on the WLAN/WiFi network.

.. figure:: hardware-setup.jpg
  :width: 70 %
  :align: center

  Hardware setup.
  
Hardware
--------

To realize the objective, following components were used:

- `Raspberry Pi 3 model B <https://www.raspberrypi.org/products/raspberry-pi-3-model-b/>`_
- `Grove Pi Plus Shield for Raspberry Pi <http://wiki.seeedstudio.com/GrovePi_Plus/>`_
- `Grove - Temperature & Humidity Sensor <http://wiki.seeedstudio.com/Grove-Temperature_and_Humidity_Sensor_Pro/>`_
- `Grove - Digital Light Sensor <http://wiki.seeedstudio.com/Grove-Digital_Light_Sensor/>`_
- Micro USB Charger

Wiring setup
------------

First of all, the `grove base shield <http://wiki.seeedstudio.com/Grove_Base_Hat_for_Raspberry_Pi/>`_ 
was connected over the Raspberry Pi board. Then, the sensor 
connections were made using the connector cables as following:

- DHT22 Sensor – Digital pin D4
- Digital Light Sensor – I2C pin-1

Apart from this, there is no need of any other wiring in this case.

Once all these connection were made, the board is `remotely accessed 
with a computer using SSH/VNC mode <https://wiki.tum.de/display/geosensorweb/Remote+access+Raspberry+Pi>`_. 
Further, steps of `software part <#software>`_ needs to be followed.

Software
--------

Configuring eduroam
^^^^^^^^^^^^^^^^^^^

To enable the raspberry pi node to connect with the eduroam 
WiFi network, first of all download this `T-Telesec Global Root Class 2 certificate <https://www.pki.dfn.de/wurzelzertifikate/globalroot2/#c18447>`_
in .pem format. Copy this certificate file in the /etc/ssl/certs/ 
folder of the Raspberry Pi.

Now, enter the following command:

.. code-block:: none
   
   sudo nano /etc/network/interfaces

Edit this file and enter the following lines in the file.
If the file is already having these lines, keep it unchanged.

.. code-block:: none
  
   allow-hotplug wlan0
   iface wlan0 inet manual
   wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf

Save the file with Ctrl+X and then press Y. Now again enter 
following command:

.. code-block:: none
  
   sudo nano /etc/wpa_supplicant/wpa_supplicant.conf

In the editor mode, enter the following lines in this file.
Modify the Identity and Password with the correct eduroam 
login credentials. Save the file with Ctrl+X and then press Y.

.. code-block:: none
  
   ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
   update_config=1
   country=DE

   network={
     ssid="eduroam"
     proto=RSN
     key_mgmt=WPA-EAP
     eap=PEAP
     ca_cert="/etc/ssl/certs/T-TeleSec_GlobalRoot_Class_2.pem"
     identity="gxxxxxx@eduroam.mwn.de"
     password="XXXXXXXXXX"
     phase1="peaplabel=0"
     phase2="auth=MSCHAPV2"
     subject_match="radius.lrz.de"}

Finally, enter the following command to restart the wifi configuration.
Reboot the RaspberryPi and it should have a running WiFi with eduroam.

.. code-block:: none
  
   sudo wpa_supplicant -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf

Installing docker
^^^^^^^^^^^^^^^^^

To install docker service on the raspberry pi follow the 
`tutorial available here <https://www.docker.com/blog/happy-pi-day-docker-raspberry-pi/>`_ 
and perform till step 4-i. After this you will have a running 
docker service on your raspberrypi.

Installing Node-Red
^^^^^^^^^^^^^^^^^^^

To install a node-red on the raspberyy pi node follow the
following `tutorial <https://help.ubidots.com/en/articles/1958375-how-to-install-node-red-in-raspberry-pi>`_
or alternatively follow the commands given below:

.. code-block:: none
   :name: node-red

   sudo apt-get install build-essential
   sudo apt-get update
   sudo apt-get upgrade
   bash <(curl -sL https://raw.githubusercontent.com/node-red/linux-installers/master/deb/update-nodejs-and-nodered)
   sudo systemctl start nodered.service      #Autostart node-red at startup
   sudo systemctl enable nodered.service      #Autostart node-red at startup

After this, you should have a running node-red service on
port 1880 which can be accessed via http://localhost:1880

It is to be **noted**, that although we have installed a node-red
service on our sensor-node, we aren't using it for this example.

Installing FROST Server
^^^^^^^^^^^^^^^^^^^^^^^

To setup a `FROST server <https://github.com/FraunhoferIOSB/FROST-Server>`_ 
follow `this detailed guide <https://github.com/FraunhoferIOSB/FROST-Server/wiki/Installation>`_ 
available on its github repository. Basically there are five major stpes:

* `Setting up Postgre SQL database <https://github.com/FraunhoferIOSB/FROST-Server/wiki/PostgreSQL-setup>`_
* `Downloading pre-compiled MQTTP FROST sever <https://bintray.com/fraunhoferiosb/Maven/download_file?file_path=de%2Ffraunhofer%2Fiosb%2Filt%2FFROST-Server%2FFROST-Server.MQTTP%2F1.10.1%2FFROST-Server.MQTTP-1.10.1.war>`_
* `Installing TOMCAT <https://github.com/FraunhoferIOSB/FROST-Server/wiki/Tomcat-Deployment>`_
*  Add the `PostgreSQL <http://repo.maven.apache.org/maven2/org/postgresql/postgresql/9.4.1212/postgresql-9.4.1212.jar>`_ and `PostGIS <http://repo.maven.apache.org/maven2/net/postgis/postgis-jdbc/2.2.1/postgis-jdbc-2.2.1.jar>`_ jars to $CATALINA_HOME/lib
*  `Deploy FROST Server on TOMCAT <https://github.com/FraunhoferIOSB/FROST-Server/wiki/Tomcat-Deployment#deploy-frost-server-in-tomcat>`_

Step-by-step commands are also provided below for the reference:

.. code-block:: none
   :name: FROST

   apt-get install postgresql postgis pgadmin3
   sudo apt-get update                                       # update package list      
   sudo apt install openjdk-11-jdk                         
   sudo apt-get install tomcat8 tomcat8-docs tomcat8-admin   # install tomcat
   sudo nano /etc/tomcat8/tomcat-users.xml
   sudo nano /usr/share/tomcat8-admin/manager/WEB-INF/web.xml
   export CATALINA_HOME=/usr/share/tomcat8
   sudo service tomcat8 restart

After the above steps are completed, a SensorThings API service 
should be running at: http://localhost:8080/FROST-Service/v1.0

GrovePi+ and Sensors
^^^^^^^^^^^^^^^^^^^^

To create this sensor node, we used Python for setting up the Raspberry Pi. 
First, install the `Dexter Grove Pi plus library <http://wiki.seeedstudio.com/Seeeduino_LoRAWAN/>`_ 
on the board. Now download and run the :ref:`all_on_one_rpi_node.py` 
file in the text editor. This code was created by merging 
the example code of each of these attached sensor with the 
python code for transmitting the data to the FROST server. 
Some required changes were made while merging the example codes, 
such as changing the pin number for the sensor. The code also 
requires following dependent libraries to run:

- :download:`Adafruit_I2C.pyc <all_on_one_rpi_node/grovepi.pyc>` for Grove Pi plus board
- :download:`Adafruit_I2C.pyc <all_on_one_rpi_node/Adafruit_I2C.pyc>` for Adafruit I2C Sensor node

Download these two .pyc files in the same folder with the 
:ref:`all_on_one_rpi_node.py`  code file. Create a sub-folder inside 
this main folder and rename it as "lib". Move the Adafruit_I2C.pyc 
into that lib folder. Now, the code can be compiled and run successfully. 
To post the sensor data to the FROST sever, an http post request
needs to be made from the python. For this we use **requests** library 
available in the python. The URL needs to be configured in the 'URL' variable.
Each post request is made separately with the unique datastream id of 
that particular sensor. Modify the post requests depending on the sensor
value and the datastream ids.

.. literalinclude:: all_on_one_rpi_node/all_on_one_rpi_node.py
   :language: python
   :linenos:
   :caption: HTTP post request in :ref:`all_on_one_rpi_node.py`
   :name: all_on_one_rpi_node.py_httppost
   :lines: 5,10-11,320-326
   :emphasize-lines: 2,5,7,9

To execute the code file run the following command:

.. code-block:: none
   :name: pipy

   python all_on_one_rpi_node.py

The code for sensors need to be modified according to the 
sensors used. The code below shows the part of the code 
used here to read, store, and print the sensor values.

.. literalinclude:: all_on_one_rpi_node/all_on_one_rpi_node.py
   :language: python
   :linenos:
   :caption: Setup the sensors in :ref:`all_on_one_rpi_node.py`
   :name: all_on_one_rpi_node.py_Sensors
   :lines: 307-319,327-329
   
Services
--------

This node direclty pushes the sensor data to the OGC Sensor Things 
API configured on the FROST Server using WLAN or WiFi connection. To 
be able to access the device from the local LAN we use the DNS service 
from the DuckDNS.

DuckDNS
^^^^^^^

`DuckDNS <http://www.duckdns.org/>`_ is a free dynamic DNS 
hosted on AWS. Anyone can create a free account on this platform 
and register an ipaddress with a free subdomain as xxxx.duckdns.org. 
For this example we registered our ipaddress as tumgispinode.duckdns.org. 
It is required to update the ip address on the website if the ip 
of the device has changed. This can be automated with a `shell script <https://www.duckdns.org/install.jsp>`_ 
on Raspberry Pi to check the ip and update it on the 
duckdns website if it has changed. This is the reference 
:ref:`duckdns.sh` used in this example. Modify the ECHO URL and the token no 
with your set-dns address and the authentication token no from the duckdns 
website.

In addition to that, a cron tab task needs to be setup for running this 
shell script every few minutes. Enter the crontab edit mode with 

.. code-block:: none
   :name: cron

    crontab -e

Now add the following lines in the end to automatically run the shell 
script every five minutes and the always run the python code for sensor 
readings on the boot. Modify the path of the files according to your 
file location.

.. code-block:: none
   :name: crontab

    */5 * * * * /home/pi/duckdns/duck.sh
    @reboot python /home/pi/GrovePi/Node/node.py 

Grafana Dashboard
^^^^^^^^^^^^^^^^^

To visualize the collected sensor data we use dashboard service 
available from `Grafana <https://grafana.com/>`_. 
To install `Grafana using docker <https://grafana.com/docs/grafana/latest/installation/docker/>`_ run: 

.. code-block:: none
   :name: Grafanadock

   $ docker run -d -p 3000:3000 grafana/grafana

To enable the Grafana service to be able to read the data from 
the OGC Sensor things API we need to use `linksmart-sensorthings-datasource <https://grafana.com/grafana/plugins/linksmart-sensorthings-datasource>`_
extension. There is another `repository <https://github.com/tum-gis/iot-frost-ecosystem/tree/master/Grafana>`_ 
explaining this installation, alternatively you install it with 
following commands:

.. code-block:: none
   :name: Grafplug

   docker exec -it -u root grafana /bin/bash
   apt-get update
   ./bin/grafana-cli plugins install linksmart-sensorthings-datasource

Datastreams setup for this sensor node on the FROST server can be seen at:
http://tumgispinode.duckdns.org:8080/FROST-Server/v1.0/Datastreams

The GRAFANA dash-board for visualizing the collected data is available at:
http://tumgispinode.duckdns.org:3000/d/NAn_6Jmgk/raspberry-pi-node?orgId=1
   
Code files
----------

.. literalinclude:: all_on_one_rpi_node/all_on_one_rpi_node.py
   :language: python
   :linenos:
   :caption: :download:`Python script for All-on-one Raspberry Pi sensor node <all_on_one_rpi_node/all_on_one_rpi_node.py>`
   :name: all_on_one_rpi_node.py

.. literalinclude:: duckdns.sh
   :language: shell
   :linenos:
   :caption: :download:`Shell script for updating IP address on duckdns platform <duckdns.sh>`
   :name: duckdns.sh
   
References
----------

- :ref:`all_on_one_rpi_node.py`
- :ref:`duckdns.sh`
- `Configure Eduroam on Raspberry Pi <https://www.elektronik-kompendium.de/sites/raspberry-pi/2205191.htm>`_
- `Install docker on Raspberry Pi <https://www.freecodecamp.org/news/the-easy-way-to-set-up-docker-on-a-raspberry-pi-7d24ced073ef/>`_
- `Fraunhofer FROST Server <https://github.com/FraunhoferIOSB/FROST-Server>`_
- `FROST Server Wiki Guide <https://github.com/FraunhoferIOSB/FROST-Server/wiki>`_
- `Duckdns auto ip address updater with a Shell script and Crontab task scheduler <https://www.duckdns.org/install.jsp>`_