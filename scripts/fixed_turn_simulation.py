import math
import random
import turtle

import matplotlib.pyplot as plt

BOUNDARY = 200
STEP = 5
MAX_COLLISIONS = 150
TURN_ANGLE_DEGREES = 100

x, y = 0.0, 0.0
angle = random.uniform(0, 360)
collision_count = 0

xs, ys = [x], [y]

screen = turtle.Screen()
screen.setup(600, 600)
screen.setworldcoordinates(-BOUNDARY - 20, -BOUNDARY - 20, BOUNDARY + 20, BOUNDARY + 20)

border = turtle.Turtle()
border.hideturtle()
border.penup()
border.goto(-BOUNDARY, -BOUNDARY)
border.pendown()
for _ in range(4):
    border.forward(2 * BOUNDARY)
    border.left(90)

bot = turtle.Turtle()
bot.hideturtle()
bot.penup()
bot.goto(x, y)
bot.pendown()
bot.speed(0)

while collision_count < MAX_COLLISIONS:
    rad = math.radians(angle)
    new_x = x + STEP * math.cos(rad)
    new_y = y + STEP * math.sin(rad)

    collided = False
    if not (-BOUNDARY <= new_x <= BOUNDARY):
        collided = True
        new_x = max(min(new_x, BOUNDARY), -BOUNDARY)
    if not (-BOUNDARY <= new_y <= BOUNDARY):
        collided = True
        new_y = max(min(new_y, BOUNDARY), -BOUNDARY)

    if collided:
        collision_count += 1
        angle = (angle - TURN_ANGLE_DEGREES) % 360

    bot.goto(new_x, new_y)
    xs.append(new_x)
    ys.append(new_y)
    x, y = new_x, new_y

turtle.done()

plt.figure(figsize=(6, 6))
plt.plot(xs, ys, linewidth=1)
plt.xlim(-BOUNDARY, BOUNDARY)
plt.ylim(-BOUNDARY, BOUNDARY)
plt.title(f"Robot trajectory with {collision_count} wall collisions FIX")
plt.xlabel("X position")
plt.ylabel("Y position")
plt.grid(True)
plt.gca().set_aspect("equal", "box")
plt.show()
