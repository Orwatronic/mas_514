##############
Web Controller
##############

.. figure:: ../figs/webapp/screenshot1.png
    :figclass: align-center

    Screenshot from the Flask web application used to control the Jetbot

This guide will explain the required steps to install the web controller feature which enables the Jetbot to be controlled from any smartphone or tablet using the touch screen. Two virtual joysticks are used as control inputs and the camera video stream is streamed to the web application background in real-time.

*******************************
Create WiFi Hotspot from Jetbot
*******************************

#. Install Linux Hotspot software:

    - :code:`sudo apt install -y libgtk-3-dev build-essential cmake gcc g++ pkg-config make hostapd`
    - :code:`git clone https://github.com/lakinduakash/linux-wifi-hotspot`
    - :code:`cd linux-wifi-hotspot/src/scripts`
    - :code:`sudo make install`

#. Clean up after installation (optional):

    - :code:`cd ~`
    - :code:`sudo rm -rd linux-wifi-hotspot`

#. Create system service:

    - Open :code:`create_ap.conf` by :code:`sudo nano /etc/create_ap.conf` and change the following entries:

        .. code-block::

	        INTERNET_IFACE=wlan0
	        SSID=jetbot<group-number>
	        PASSPHRASE=jetbot<group-number>-pwd

    - Open/create the service file by executing :code:`sudo nano /etc/systemd/system/create_ap.service` and paste the following code into the nano editor and save:

        .. code-block::

            [Unit]
            Description=Create AP Service
            After=network.target

            [Service]
            Type=simple
            ExecStart=/usr/bin/create_ap --config /etc/create_ap.conf
            KillSignal=SIGINT
            Restart=on-failure
            RestartSec=5

            [Install]
            WantedBy=multi-user.target

#. Enable service to be automatically started after boot/startup:
    - :code:`sudo systemctl enable create_ap`

#. Start service
    - :code:`sudo systemctl start create_ap`

#. See status of system service
    - :code:`sudo systemctl status create_ap`



***********************
Install Web Application
***********************

Actions done:
- Install Flask: pip install Flask
- Add folders static and templates to ./src
- Add WebController.py to ./src and sudo chmod +x WebController.py
- Add WebJoystick.msg to ./msg
- Add WebJoystick.msg in CMakeLists.txt under add_message_files()
- Execute cakin_make from ~/catkin_ws folder
- Add <node name="WebController" pkg="mas507" type="WebController.py" output="screen"/> to start.launch file
- Visit http://jetbot-desktop<group-number>:8000

