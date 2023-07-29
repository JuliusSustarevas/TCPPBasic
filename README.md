TCPPBasic

# 1 Download raw data
+RM/data/download_files.bash should download raw mat files. These are raw data related to creationg of RM/IRM structures. In general, they contain end effector poses and resulting IK solution

# 1.1
Go to +Tasks/specific_tasks/HelloWorldConstant (or varying) and run that script. On prompt select the test world yaml file in m3dp_scenes
# 2 RRT Entry Point
Try to read comments/run +RRT/entry_point.  

# 3 ???

# 4 Profit

# Notes:
Main places that are interesting for RM/IRM perspective are the classes +RM/ASRM +RM/ASIRM and the +RRT/Scene. 
Main functions to look out for are point2ri and point2iri. These are the main " get me the (Inverse) Reachability Index" functions
From Sampling perspective -> look at Scene.sample_irm and ASIRM.sample_irm_at

... best of luck :)

P.S. This code is a quick attempt at cleaning up 5 year long Phd project. it is by no means readable/usable :)