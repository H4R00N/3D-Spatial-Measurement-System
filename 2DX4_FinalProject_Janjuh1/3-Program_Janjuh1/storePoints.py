import serial#needed to read from port
import render3D #py code I made to render 3D object from data

if __name__ == '__main__':#acts like a main function
    totalSlices = slices = 5#this is the total slices, slices will act as a counter

    # Opening the serial input and opening a new file to store points in
    s = serial.Serial("COM5", 115200)
    f = open("ToFInitDetails.txt","w+")#stores initilization results in file

    print("Opening: " + s.name)
    line = s.readline().decode()
    #ToF init data
    while line.find("Start\r")==-1:#will store all info in initialization file until it sees a start
        print(line)
        f.write(line)
        line = s.readline().decode()#waits to get next UART serial data and decodes it
    f.close();

    f = open("points2dx4proj.xyz","w+")

    while True:#runs untill while loop is broken
        print(line)
        if line.find("End\r")!=-1:
            slices-=1;
        elif line.find("Start\r")==-1:
            f.write(line)
        if slices == 0:#only breaks when code has done 5 slices of measurements
            break
        line = s.readline().decode()#if code didnt break read next data and decode it

    f.close()
    s.close()

    render3D.renderFile(totalSlices)#use other python program provided to 3d render of newly stored data
