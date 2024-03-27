# Software-Development

## Installation of the VM for the final project
For this project you need to get a licence key of Windows 10/11 educational so you can have access to the hyper-v (hypervisor to create virtual machines). To get the key: https://www.surfspot.nl/gratis-windows-11-upgrade-education-student.html/

### SSH key generation
To access the virtual machine remotely you will need a SSH key. To create one open the terminal and execute the folloing command
```powershell
TODO
```

### Download Multipass
To download multipass just go to the following link and follow the recommended step. Important, don't forget to select the Hyper-v in the hypervisor selection

### Creation of the virtual machine 
To create the virtual machine you will need to go to the location of the cloud-init.yaml. Open a terminal and execute the following command
```powershell
multipass launch ros2-humble --name humble --timeout 600 --cpus 4 --memory 4G --cloud-init cloud-init.yaml
```

## To use Git
Download it from here: https://git-scm.com/
### To upload the changes use the following commands 
#### Add the changes to the commit
```bash
$ git add [name of the file] 
```
#### Commit the changes
```bash
$ git commit -m [Commit description]
```

#### Push the changes to main
```bash
$ git push
```

## Compilation
To compile the projects use the fllowing command
```bash
g++ -o [name of .exe] [file].cpp [file2].cpp ....
```
