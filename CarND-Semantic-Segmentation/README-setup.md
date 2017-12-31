
1. Install missing python module in AMI
   ```
   pip install tqdm
   ```

2. Reinstall nvidia drivers, because driver in AMI seems to be outdated for the new g3.4xlarge VM that AWS provides
   ```	     
   sudo apt-get remove nvidia-*
   wget http://us.download.nvidia.com/XFree86/Linux-x86_64/367.57/NVIDIA-Linux-x86_64-367.57.run
   sudo bash ./NVIDIA-Linux-x86_64-367.57.run  --dkms
   ```

3. Periodically backup files from instance to local machine
   ```
   scp -i ~/.aws/udacity-dl-default.pem ubuntu@ec2-18-220-42-205.us-east-2.compute.amazonaws.com:/home/ubuntu/CarND-Semantic-Segmentation/main.py .
   ```