------------------------------------------------------------------------
--   Global Variables
------------------------------------------------------------------------
package.path = package.path .. ";../?.lua"
package.path = package.path .. ";../math/?.lua"
package.path = package.path .. ";../VNSModules/?.lua"
--require("debugger")
require("TableTools")

local IF = {} -- Robot Interface
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")

local VNS = require("VNS")

VNS.EnableModules = {
	VNS.Modules.VehicleConnector,
	VNS.Modules.QuadcopterConnector,
	VNS.Modules.LostCounter,

	--VNS.Modules.Assigner,
	--VNS.Modules.Maintainer,
	--VNS.Modules.ShiftUpper,
	VNS.Modules.Shifter,

	VNS.Modules.RandomWalker,
	VNS.Modules.Driver,
}

local vns

------------------------------------------------------------------------
--   ARGoS Functions
------------------------------------------------------------------------
function init()
	local dis = 100
	local structure = {
		locV3 = Vec3:create(),
		dirQ = Quaternion:create(),
		children = {
			--[[
			{	robotType = "vehicle",
				locV3 = Vec3:create(dis, dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},
			{	robotType = "vehicle",
				locV3 = Vec3:create(dis, -dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},
			--]]
			{	robotType = "vehicle",
				locV3 = Vec3:create(-dis, -dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},
			{	robotType = "vehicle",
				locV3 = Vec3:create(-dis, dis, 0),
				dirQ = Quaternion:create(0,0,1, 0),
			},
			{	robotType = "quadcopter",
				locV3 = Vec3:create(-dis*2, 0, 0),
				dirQ = Quaternion:create(0,0,1, 0),
				children = {
					{	robotType = "vehicle",
						locV3 = Vec3:create(-dis, -dis, 0),
						dirQ = Quaternion:create(0,0,1, 0),
					},
					{	robotType = "vehicle",
						locV3 = Vec3:create(-dis, dis, 0),
						dirQ = Quaternion:create(0,0,1, 0),
					},
					--[[
					{	robotType = "quadcopter",
						locV3 = Vec3:create(-dis*2, dis*2, 0),
						dirQ = Quaternion:create(0,0,1, 0),
					},
					{	robotType = "quadcopter",
						locV3 = Vec3:create(-dis*2, 0, 0),
						dirQ = Quaternion:create(0,0,1, 0),
					},
					--]]
				},
			},
			{	robotType = "quadcopter",
				locV3 = Vec3:create(0,dis*2, 0),
				dirQ = Quaternion:create(0,0,1, -math.pi/2),
				children = {
					{	robotType = "vehicle",
						locV3 = Vec3:create(-dis, -dis, 0),
						dirQ = Quaternion:create(0,0,1, 0),
					},
				--[[
					{	robotType = "vehicle",
						locV3 = Vec3:create(-dis, dis, 0),
						dirQ = Quaternion:create(0,0,1, 0),
					},
					{	robotType = "quadcopter",
						locV3 = Vec3:create(-dis*2, dis*2, 0),
						dirQ = Quaternion:create(0,0,1, 0),
					},
				--]]
				},
			},
		},
	}

	vns = VNS:new{id = IF.myIDS()}
	vns.modules[4]:setStructure(vns, structure)

	if IF.myIDS() == "quadcopter0" then
		vns.modules[5] = vns.modules[6]
		vns.modules[6] = nil
	end
	reset()
end

-------------------------------------------------------------------
function reset()
	vns:reset()
end

-------------------------------------------------------------------
function step()
	--print("----------" .. IF.myIDS() .. "------------------")

	vns:run{vehiclesTR = getVehicleTR()}
	print("parent = ", vns.parentS)
	print("childrenTVns = ")
	showTable(vns.childrenTVns, 1, "modules")
	for i, v in pairs(vns.childrenTVns) do
		print("\t", i)
	end
	print("AssignTable")
	showTable(vns.modules[4].childrenAssignTS, 1)
	print("myAssignParent", vns.myAssignParent)
end

-------------------------------------------------------------------
function destroy()
end

------------------------------------------------------------------------
--   Customize Functions
------------------------------------------------------------------------

function getVehicleTR()
	local vehicleTR = {}
	for i, tagDetectionT in pairs(IF.getTagsT()) do
		-- a tag detection is a complicated table
		-- containing center, corners, payloads
		
		-- get robot info
		local locV3, dirQ, idS = getVehicleInfo(tagDetectionT)
			-- locV3 for vector3 {x,y,z}
			-- loc (0,0) in the middle, x+ right, y+ up , 
			-- dir a quaternion, x+ as 0

		vehicleTR[idS] = {locV3 = locV3, dirQ = dirQ, idS = idS, }
	end
	return vehicleTR
end

function getVehicleInfo(tagT)
	-- a tag is a complicated table with center, corner, payload
	local degQ = calcVehicleDir(tagT.corners)
		-- a direction is a Quaternion
		-- with 0 as the x+ axis of the quadcopter
		
	local x = tagT.center.x - 320
	local y = tagT.center.y - 240
	y = -y 				-- make it left handed coordination system
	local z = 0
	local locV3 = Vec3:create(x,y,z)

	local idS = tagT.payload

	return locV3, degQ, idS
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
	return deg
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

VNS.move = function(transV3, rotateV3)
	local speedscale = 0.15
	local turnscale = 1
	local x = transV3.x * speedscale
	local y = transV3.y * speedscale
	local w = rotateV3:len() * turnscale
	if rotateV3.z < 0 then w = -w end
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

function IF.setVelocity(x, y, w)
	--quadcopter heading is the x+ axis
	local thRad = robot.joints.axis2_body.encoder
	local xWorld = x * math.cos(thRad) - y * math.sin(thRad)
	local yWorld = x * math.sin(thRad) + y * math.cos(thRad)
	robot.joints.axis0_axis1.set_target(xWorld)
	robot.joints.axis1_axis2.set_target(yWorld)
	robot.joints.axis2_body.set_target(w)
end

