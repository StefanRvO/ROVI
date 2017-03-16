#Dockerfile for running the project in docker...
FROM registry.stefanrvo.dk/robwork:latest

#get the code
RUN git clone https://github.com/StefanRvO/ROVI.git
RUN mkdir ROVI/Final_project/build
RUN cd ROVI/Final_project/build && cmake ..
RUN cd ROVI/Final_project/build && make

#Copy stuff to /home/student/Downloads
USER root
RUN mkdir /home/student
RUN chown rw_user:rw_user /home/student -R
USER rw_user

ENTRYPOINT [ "RobWork/RobWorkStudio/bin/release/RobWorkStudio" ]

