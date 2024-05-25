Because the file uploaded to github is limited within 100M, so I have splited the files into several parts. You can use the commands bellow speperately to unzip the whole file.

1.  cat CH7_slide.tar.gza* > CH7_slide.tar.gz
Description: Combine each volume compressed package into a proc.tar.gz file.

2.  tar -zxvf  CH7_slide.tar.gz
Instructions: Unzip the proc.tar.gz file to the current directory.
