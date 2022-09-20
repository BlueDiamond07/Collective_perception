local Vec3 = require("Vector3")
local Quad = require("Quaternion")

local Linar = {}
function Linar.myVecToYou(myVec, yourLocation, yourQuaternion)
	local relativeLoc = myVec - yourLocation
	return yourQuaternion:inv():toRotate(relativeLoc)
end

function Linar.mySpeedToYou(mySpeedV3, yourQuaternion)
	return yourQuaternion:inv():toRotate(mySpeedV3)
end

function Linar.myQuadToYou(myQuad, yourQuaternion)
	return myQuad * yourQuaternion:inv()
end

function Linar.yourVecToMe(yourVec, yourLocation, yourQuaternion)
	return yourQuaternion:toRotate(yourVec) + yourLocation
end

function Linar.yourLocBySameObj(myVec, myQua, yourVec, yourQua)
	local yourDirQ = Linar.yourDirBySameObj(myQua, yourQua)
	return yourDirQ:toRotate(-yourVec) + myVec
end

function Linar.yourDirBySameObj(myQua, yourQua)
	return myQua * yourQua:inv()
end

return Linar
