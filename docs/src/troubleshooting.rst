Troubleshooting
===============

Can't Connect to Jetbot using SSH
---------------------------------
If the connection is not working, check the following steps:

- Is the private SSH key located in your :code:`.ssh` folder?
- Are the SSH config using the correct IP address?
- Can you log in using Putty?
- If you can log in using Putty, try to execute :code:`sudo rm -rd ~/.vscode-server` and then try to log in using VS Code again.