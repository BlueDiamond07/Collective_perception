------------------------------------------------------------------------
--   Global Variables
------------------------------------------------------------------------
package.path = package.path .. ";math/?.lua"
package.path = package.path .. ";VNSModules/?.lua"
package.path = package.path .. ";testing/?.lua"

require("TableTools")

local IF = {} -- Robot Interface
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")

local VNS = require("VNS")

VNS.EnableModules = {
	VNS.Modules.VehicleConnector,
	VNS.Modules.QuadcopterConnector,
	VNS.Modules.LostCounter,
	VNS.Modules.Shifter,
	VNS.Modules.RandomWalker,
	VNS.Modules.Driver,
}

local vns

------------------------------------------------------------------------
--   ARGoS Functions
------------------------------------------------------------------------
function init()

	local structure = {
		locV3 = Vec3:create(),
		dirQ = Quaternion:create(),
		children = {
	
		},
	}

	vns = VNS:new{idS = IF.myIDS(), robotType = "quadcopter"}

	vns.modules[4]:setGene(vns, structure)

	reset()
end

-------------------------------------------------------------------
function reset()
	vns:reset()
end

-------------------------------------------------------------------
function step()

end

-------------------------------------------------------------------
function destroy()
end

------------------------------------------------------------------------
--   Customize Functions
------------------------------------------------------------------------
function detectCorner()
	local array = {}
	local cornerflag = 0
	for i, detectionT in ipairs(IF.getLEDsT()) do	
		print(detectionT.center.x, detectionT.center.y)
		array[i]={x = detectionT.center.x, y = detectionT.center.y}
		if ((detectionT.color.blue == 255) and detectionT.center.x>240 and detectionT.center.x<=390) then
			cornerflag = 1
		end

		if cornerflag == 1 and detectionT.color.blue == 255 then
			for i=1, #array do
				for j=i+1, #array do
					if (array[i].x > 490 and array[i].x > 490) and math.abs(array[i].x-array[j].x) <= 1 then
						 cornerflag = 0	
		
					end
				end
			end
		end	
	end
	
	return cornerflag
end

function getVehicleTR()
	local vehicleTR = {}
	for i, tagDetectionT in pairs(IF.getTagsT()) do
		-- a tag detection is a complicated table
		-- containing center, corners, payloads
		
		-- get robot info
		local locV3, dirQ, idS, front = getVehicleInfo(tagDetectionT)
			-- locV3 for vector3 {x,y,z}
			-- loc (0,0) in the middle, x+ right, y+ up , 
			-- dir a quaternion, x+ as 0

		vehicleTR[idS] = {locV3 = locV3, dirQ = dirQ, idS = idS, front = front}
	end
	return vehicleTR
end

function getVehicleInfo(tagT)
	-- a tag is a complicated table with center, corner, payload
	local degQ, front = calcVehicleDir(tagT.corners)
		-- a direction is a Quaternion
		-- with 0 as the x+ axis of the quadcopter
	front.x = front.x - 320
	front.y = -(((tagT.corners[3].y + tagT.corners[4].y) / 2)-240) 	
	local x = tagT.center.x - 320
	local y = tagT.center.y - 240
	y = -y 				-- make it left handed coordination system
	local z = 0
	local locV3 = Vec3:create(x,y,z)
	local idS = tagT.payload

	return locV3, degQ, idS, front
end

function calcVehicleDir(corners)
	-- a direction is a number from 0 to 360, 
	-- with 0 as the x+ axis of the quadcopter
	local front = {}
	front.x = (corners[3].x + corners[4].x) / 2
	front.y = -(corners[3].y + corners[4].y) / 2
	local back = {}
	back.x = (corners[1].x + corners[2].x) / 2
	back.y = -(corners[1].y + corners[2].y) / 2
	local deg = calcDir(back, front)
	return deg, front
end

function calcDir(center, target)
	-- from center{x,y} to target{x,y} in left hand
	-- return a Quaternion
	local x = target.x - center.x
	local y = target.y - center.y
	local rad = math.atan(y / x)
	if x < 0 then
		rad = rad + math.pi
	end
	if rad < 0 then
		rad = rad + math.pi*2
	end
	return Quaternion:create(0,0,1, rad)
end

function getBoxesTR()
	local boxesTR = {}   -- vt for vector, which a table = {x,y}
	for i, ledDetectionT in ipairs(IF.getLEDsT()) do	
		-- a detection is a table
		-- {center = {x,y}, color = {blue, green, red}}
		local locV3, dirQ, colorS = getBoxInfo(ledDetectionT)
		boxesTR[i] = {locV3 = locV3, dirQ = dirQ, colorS = colorS}
	end	
	return boxesTR
end

function getBoxInfo(detection)
	local x = detection.center.x - 320
	local y = detection.center.y - 240
	y = -y
	local z = 0
	local locV3 = Vec3:create(x,y,z)
	local dirQ = Quaternion:create()
	local colorS = "red"
	return locV3, dirQ, colorS
end

function getPredatorsTR()
	local predatorsTR = {}   -- vt for vector, which a table = {x,y}
	local j = 0
	for i, ledDetectionT in ipairs(IF.getLEDsT()) do	
		if ledDetectionT.color.blue > 150 or ledDetectionT.color.green > 150 or ledDetectionT.color.red > 150 then -- else continue
		j = j + 1
		-- a detection is a table
		-- {center = {x,y}, color = {blue, green, red}}
		local locV3, dirQ, colorS = getPredatorInfo(ledDetectionT)
		predatorsTR[j] = {locV3 = locV3, dirQ = dirQ, colorS = colorS}
	end	end
	return predatorsTR
end

function getPredatorInfo(detection)
	local x = detection.center.x - 320
	local y = detection.center.y - 240
	y = -y
	local z = 0
	local locV3 = Vec3:create(x,y,z)
	local dirQ = Quaternion:create()
	local colorS -- = "blue" 
	if detection.color.blue > 150 and detection.color.red == 0 then
		colorS = "blue"
	elseif detection.color.green > 150 then
		colorS = "green"
	elseif detection.color.red > 150 and detection.color.blue == 0 then
		colorS = "red"
	end
	return locV3, dirQ, colorS
end

------------------------------------------------------------------------
--   VNS Callback Functions
------------------------------------------------------------------------
VNS.Msg.myIDS = function()
	return IF.myIDS()
end

VNS.Msg.Packet.sendData = function(dataAN)
	robot.radios["radio_0"].tx_data(dataAN)	
end

VNS.Msg.Packet.receiveDataAAN = function()
	return robot.radios["radio_0"].rx_data
end

VNS.move = function(transV3, rotateV3, time, energy)
	local speedscale = 0.15
	local turnscale = 1
	local x = transV3.x * speedscale
	local y = transV3.y * speedscale
	local w = rotateV3:len() * turnscale
	if rotateV3.z < 0 then w = -w end
	IF.setVelocity(x, y, w)

end

VNS.speed = function(x, y, w)
	IF.setVelocity(x, y, w)
end
------------------------------------------------------------------------
--  Robot Interface 
------------------------------------------------------------------------
function IF.myIDS()
	return robot.id
end

function IF.getTagsT()
	return robot.cameras.fixed_camera.tag_detector
end

function IF.getLEDsT()
	return robot.cameras.fixed_camera.led_detector
end

function IF.setVelocity(x, y, w)
	--quadcopter heading is the x+ axis
	local thRad = robot.joints.axis2_body.encoder
	local xWorld = x * math.cos(thRad) - y * math.sin(thRad)
	local yWorld = x * math.sin(thRad) + y * math.cos(thRad)
	robot.joints.axis0_axis1.set_target(xWorld)
	robot.joints.axis1_axis2.set_target(yWorld)
	robot.joints.axis2_body.set_target(w)
end

