import math

angles = []

for x in range(16):
    for y in range(16):

        angle = math.degrees(math.atan2(x, y))
        print("X : " + str(x) + " Y : " + str(y) + " => Angle : " + str(angle))

        angles.append(angle)


angles_uniques = sorted(set(angles))
differences = [angles_uniques[i+1] - angles_uniques[i] for i in range(len(angles_uniques) - 1)]


print((differences))

moyenne = sum(differences) / len(differences)

print(moyenne)
