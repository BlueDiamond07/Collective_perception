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
	stepCounter = 0
	local dis = 30
	-- the following structrue is used in case of using one UAV	
	local structure = {
		locV3 = Vec3:create(),
		dirQ = Quaternion:create(),
		children = {

			{	robotType = "quadcopter",
				locV3 = Vec3:create(0, -50, 0),
				dirQ = Quaternion:create(0,0,1, 0),
				children = {
					{	robotType = "vehicle",
						locV3 = Vec3:create(0.3*dis, 0.3*dis, 0),
						dirQ = Quaternion:create(0,0,1, 0),
					},
					{	robotType = "vehicle",
						locV3 = Vec3:create(-0.25*dis, -0.3*dis, 0),
						dirQ = Quaternion:create(0,0,1, 0),
					},					
					
				},
			},
			
			{	robotType = "vehicle",
				locV3 = Vec3:create(0.3*dis, -1.4*dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},
			{	robotType = "vehicle",
				locV3 = Vec3:create(0.3*dis,2.1*dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},

			{	robotType = "vehicle",
				locV3 = Vec3:create(-0.25*dis,1.4*dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},

			{	robotType = "vehicle",
				locV3 = Vec3:create(-0.25*dis,-2.1*dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},
			{	robotType = "quadcopter",
				locV3 = Vec3:create(0, 50, 0),
				dirQ = Quaternion:create(0,0,1, 0),
				children = {
					{	robotType = "vehicle",
						locV3 = Vec3:create(0.3*dis, 0.3*dis, 0),
						dirQ = Quaternion:create(0,0,1, 0),
					},
					{	robotType = "vehicle",
						locV3 = Vec3:create(-0.25*dis, -0.3*dis, 0),
						dirQ = Quaternion:create(0,0,1, 0),
					},					
				},
			},
			{	robotType = "vehicle",
				locV3 = Vec3:create(-0.25*dis, 0.3*dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},
			{	robotType = "vehicle",
				locV3 = Vec3:create(0.3*dis, -0.3*dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},
			{	robotType = "vehicle",
				locV3 = Vec3:create(0.3*dis,1*dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},

			{	robotType = "vehicle",
				locV3 = Vec3:create(-0.25*dis,-1*dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},
					
			
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
	print("----------" .. IF.myIDS() .. "------------------")
	stepCounter = stepCounter+1    -- increase time step by one at each step
	vns:run{vehiclesTR = getVehicleTR(), 
	        boxesTR = getBoxesTR(),
	        predatorsTR = getPredatorsTR(), cornerN = detectCorner(), stepCounter = stepCounter, energyN = IF.getenergy(), myid = IF.myIDS()}	
end

-------------------------------------------------------------------
function destroy()
end

------------------------------------------------------------------------
--   Customize Functions
------------------------------------------------------------------------
function detectCorner()    -- this function is used for detecting the border from front of the MNS, from back of the MNS, or for detecting the white LEDs placed on the corners of the arena
	local cornerflag = 0
	for i, detectionT in ipairs(IF.getLEDsT()) do
		-- for detecting a white LED in the specified range from the brain (brain's camera)	
		if (vns.parentS == nil  and detectionT.color.green == 255 and detectionT.color.red == 255 and detectionT.color.blue == 255 and ((detectionT.center.x>=281 and detectionT.center.x<=308) or (detectionT.center.x>=350 and detectionT.center.x<=360)) and detectionT.center.y < 304 and detectionT.center.y > 175) then
			if (stepCounter > 13500 and stepCounter < 20000) or (stepCounter > 26500 and stepCounter < 30000) or stepCounter > 41000 then    -- these conditions are used for implementing the zig-zag motion of the MNS
				cornerflag = 3
				break
			end
		-- for detecting a border (yellow LEDs) from the front of the MNS, in the specified range from the brain (brain's camera)
		elseif (vns.parentS == nil  and detectionT.color.green == 255 and detectionT.color.red == 255 and detectionT.center.x>=350 and detectionT.center.x<=360) then 
			cornerflag = 1
		-- for detecting a border (yellow LEDs) from the back of the MNS, in the specified range from the brain (brain's camera)
		elseif (stepCounter > 500 and vns.parentS == nil  and detectionT.color.green == 255 and detectionT.color.red == 255 and detectionT.center.x>=281 and detectionT.center.x<=308) then 
			cornerflag = 2
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
		local locV3, dirQ, idS, front= getVehicleInfo(tagDetectionT)
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

function getBoxesTR()    -- this function is not used
	local boxesTR = {}   -- vt for vector, which a table = {x,y}
	for i, ledDetectionT in ipairs(IF.getLEDsT()) do	
		if (ledDetectionT.color.red ~= 255 and ledDetectionT.color.green ~= 255) and (ledDetectionT.color.blue ~= 0 or ledDetectionT.color.red ~= 0 or ledDetectionT.color.green ~= 0) then
			local locV3, dirQ, colorS = getBoxInfo(ledDetectionT)
			boxesTR[i] = {locV3 = locV3, dirQ = dirQ, colorS = colorS}
		end
	end	
	return boxesTR
end

function getBoxInfo(detection)    -- this function is not used
	local x = detection.center.x - 320
	local y = detection.center.y - 240
	y = -y
	local z = 0
	local locV3 = Vec3:create(x,y,z)
	local dirQ = Quaternion:create()
	local colorS = "orange"
	if (detection.color.blue == 0 and detection.color.red == 0 and detection.color.green == 255) then
		colorS = "green"
	elseif (detection.color.blue == 0 and detection.color.red == 255 and detection.color.green == 0) then
		colorS = "red"
	elseif (detection.color.blue == 255 and detection.color.red == 0 and detection.color.green == 0) then
		colorS = "blue"
	elseif (detection.color.blue > 0 and detection.color.red > 0 and detection.color.green ~= 255) then
		colorS = "purple"				
	end
	return locV3, dirQ, colorS
end

function getPredatorsTR()    -- this function is not used
	local predatorsTR = {}   -- vt for vector, which a table = {x,y}
	local j = 0
	for i, ledDetectionT in ipairs(IF.getLEDsT()) do	
		if ledDetectionT.color.blue > 150 then -- else continue
		j = j + 1
		-- a detection is a table
		-- {center = {x,y}, color = {blue, green, red}}
		local locV3, dirQ, colorS = getPredatorInfo(ledDetectionT)
		predatorsTR[j] = {locV3 = locV3, dirQ = dirQ, colorS = colorS}
	end	end
	return predatorsTR
end

function getPredatorInfo(detection)    -- this function is not used
	local x = detection.center.x - 320
	local y = detection.center.y - 240
	y = -y
	local z = 0
	local locV3 = Vec3:create(x,y,z)
	local dirQ = Quaternion:create()
	local colorS = "blue"
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

-- this function is called only when the MNS is moving forward or backward	
VNS.move = function(transV3, rotateV3, time, energy)
	local speedscale = 0.15
	local turnscale = 1
	local x = transV3.x * speedscale
	local y = transV3.y * speedscale
	local w = rotateV3:len() * turnscale
	if rotateV3.z < 0 then w = -w end
	-- depending on the movement direction, the speed is equal to either 7.5 cm/s or -7.5 cm/s (i.e., either x = 0.075 or x = -0.075) when this function is called	
	IF.setVelocity(x, y, w)
end

function IF.setenergy(m)    -- this function is not used
	energy = math.abs(m)
	local file = io.open ("energy.csv","w" )
	file:write(energy)
	file:close()
end

function IF.getenergy()    -- this function is not used
	local file = io.open ("energy.csv","r" )
	io.input(file)
	local energy = io.read()
	return energy
end

-- this function is called during the time that the brain is waiting after detecting a border or during the time that the MNS is shifting
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
