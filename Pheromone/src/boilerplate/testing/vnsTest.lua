package.path = package.path .. ";../math/?.lua"
package.path = package.path .. ";../?.lua"
local VNS = require("VNS")

vns = VNS:new{id = "id"}
vns:run()
