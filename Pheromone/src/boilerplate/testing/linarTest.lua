package.path = package.path .. ";../math/?.lua"
Linar = require("Linar")
Vec3 = require("Vector3")
Quad = require("Quaternion")

a = Vec3:create(0,1,0);

obj = {loc = Vec3:create(1,1,0),
	   quad = Quad:create(0,0,1,math.pi/4)}

print(Linar.myVecToYou(a, obj.loc, obj.quad))
print(Linar.yourVecToMe(a, obj.loc, obj.quad))

b = Quad:create(0,0,1, math.pi/2)

print("myQuad rotate (1,0,0) = ", b:toRotate(Vec3:create(1,0,0)))

b_ = Linar.myQuadToYou(b, obj.quad)
print("yourQuad rotate (1,0,0) = ", b_:toRotate(Vec3:create(1,0,0)))
