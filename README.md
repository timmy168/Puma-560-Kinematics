# Puma 560 Kinematics README

# DH Model and Angle Constraints

---

![Untitled](Puma%20560%20Kinematics%20README%209dcd5fe4475b40548c43bd28e959f4d3/Untitled.png)

# Oeration

---

- Open the code

## Foward Kinematics

---

- Input 6 joint angles in degree as the form : `[theta1,theta2,theta3,theta4,theta5,theta6]`

![Untitled](Puma%20560%20Kinematics%20README%209dcd5fe4475b40548c43bd28e959f4d3/Untitled%201.png)

- Get the result

![Untitled](Puma%20560%20Kinematics%20README%209dcd5fe4475b40548c43bd28e959f4d3/Untitled%202.png)

## Inverse Kinematics

---

- Input the transformation matrix to the 6th joint

![Untitled](Puma%20560%20Kinematics%20README%209dcd5fe4475b40548c43bd28e959f4d3/Untitled%203.png)

- Get 8 solutions of the 6 joint angles

![Untitled](Puma%20560%20Kinematics%20README%209dcd5fe4475b40548c43bd28e959f4d3/Untitled%204.png)