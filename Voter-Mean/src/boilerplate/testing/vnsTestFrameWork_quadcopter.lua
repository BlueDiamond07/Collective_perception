------------------------------------------------------------------------
--   Global Variables
------------------------------------------------------------------------
package.path = package.path .. ";../?.lua"
package.path = package.path .. ";../math/?.lua"
--require("debugger")
require("TableTools")

local IF = {} -- Robot Interface
local Vec3 = require("Vector3")
local Quaternion = require("Quaternion")

local VNS = require("VNS")

VNS.runModules = {
	--VNS.modules.parentwaitor,
	VNS.modules.vehicleConnector,
	--VNS.modules.driver,
}

local vns

------------------------------------------------------------------------
--   ARGoS Functions
------------------------------------------------------------------------
function init()
	reset()
end

-------------------------------------------------------------------
function reset()
	vns = VNS:new{id = IF.myIDS()}
end

-------------------------------------------------------------------
function step()
	print("----------" .. IF.myIDS() .. "------------------")

	vns:run{vehiclesTR = getVehicleTR()}

	showTable(vns)
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
	local deg = math.atan(y / x) * 180 / 3.1415926
	if x < 0 then
		deg = deg + 180
	end
	if deg < 0 then
		deg = deg + 360
	end
	return Quaternion:create(0,0,1, deg)
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

------------------------------------------------------------------------
--  Robot Interface 
------------------------------------------------------------------------
function IF.myIDS()
	return robot.id
end

function IF.getTagsT()
	return robot.cameras.fixed_camera.tag_detector
end
