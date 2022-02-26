import turtle
from PIL import Image
import time
import csv

speed = 0.1

turtle.setup(900, 600)
turtle.tracer(False)
turtle.bgcolor("gray15")
turtle.title("SmartCar Simulation GUI")

img_turtle = turtle.Turtle()
img_turtle.up()
img_turtle.goto(-150, 0)
org_turtle = turtle.Turtle()
org_turtle.up()
org_turtle.goto(200, 120)
code_turtle = turtle.Turtle()
code_turtle.up()
code_turtle.hideturtle()
code_turtle.goto(120, -150)
code_turtle.color("white")

with open('output.csv', newline='') as csvfile:
    rows = csv.reader(csvfile)
    header = []
    first_line = True
    for row in rows:
        if first_line:
            first_line = False
            header = row
        else:
            im = Image.open("Output/" + row[0] + ".bmp")
            code_turtle.clear()
            running_data = ""
            for i in range(len(header)):
                running_data += "\n" + header[i] + ": " + row[i]
            code_turtle.write(running_data, font=("Arial", 18, "bold"))
            im.save("gui_buff1.gif")
            turtle.addshape("gui_buff1.gif")
            img_turtle.shape("gui_buff1.gif")

            im = Image.open("Greyscale/" + row[0] + ".bmp")
            im = im.resize((160, 120),Image.ANTIALIAS)
            im.save("gui_buff2.gif")
            turtle.addshape("gui_buff2.gif")
            org_turtle.shape("gui_buff2.gif")
            turtle.update()
            time.sleep(speed)

turtle.done()
