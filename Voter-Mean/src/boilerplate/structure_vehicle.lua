------------------------------------------------------------------------
--   Global Variables
------------------------------------------------------------------------
package.path = package.path .. ";math/?.lua"
package.path = package.path .. ";VNSModules/?.lua"
package.path = package.path .. ";testing/?.lua"

require("TableTools")

local IF = {} -- Robot Interface
local VNS = require("VNS")

VNS.EnableModules = {
	VNS.Modules.RandomWalker,
	VNS.Modules.BoxAvoider,
	VNS.Modules.Driver,
}

local vns

------------------------------------------------------------------------
--   ARGoS Functions
------------------------------------------------------------------------
function init()
	vns = VNS:new{idS = IF.myIDS(), robotType = "vehicle"}
	reset()
	getBoxCount = nil
	stepCounter = 0
	state = 0
	My_Cumulative_BoxCounter = {}
	Average_Cumulative_BoxCounter = {} -- not used
	MostRecent_Received_Opinion_Time = {}
	myOpinions = {}
	The_List_Average = 0
	----------------------------------------
	name = IF.myIDS() 
	local file = io.open ("Robots-Memories/Others/"..name..".csv","w" ) -- not used
	file:write(0)
	file:close()
	----------------------------------------
	local file = io.open ("Robots-Memories/Others/info.csv","w" ) -- not used
	--file:write(0)
	file:close()
		
	------------------------------------------------------------------------
	--  CSV file structure
	------------------------------------------------------------------------
	if IF.myIDS() == 'vehicle0' then
		local file = io.open ("Robots-Memories/Results/info_V0.csv","w" )  -- creating the csv file used to record the outputs corresponding to vehicle0
		file:close()
		local file_ave_1000 = io.open ("Robots-Memories/Others/ave_1000.csv","w" ) -- not used
		file_ave_1000:write(0)
		file_ave_1000:close()
		local file_ave_overall = io.open ("Robots-Memories/Others/ave_overall.csv","w" ) -- not used
		file_ave_overall:write(0)
		file_ave_overall:close()
		local file_rand_1000 = io.open ("Robots-Memories/Others/rand_1000.csv","w" ) -- not used
		file_rand_1000:write(0)
		file_rand_1000:close()
		local file_rand_overall = io.open ("Robots-Memories/Others/rand_overall.csv","w" ) -- not used
		file_rand_overall:write(0)
		file_rand_overall:close()
	elseif IF.myIDS() == 'vehicle1' then
		local file = io.open ("Robots-Memories/Results/info_V1.csv","w" ) -- creating the csv file used to record the outputs corresponding to vehicle1
		file:close()
	elseif IF.myIDS() == 'vehicle2' then
		local file = io.open ("Robots-Memories/Results/info_V2.csv","w" ) -- creating the csv file used to record the outputs corresponding to vehicle2
		file:close()
	elseif IF.myIDS() == 'vehicle3' then
		local file = io.open ("Robots-Memories/Results/info_V3.csv","w" ) -- creating the csv file used to record the outputs corresponding to vehicle3
		file:close()
	elseif IF.myIDS() == 'vehicle4' then
		local file = io.open ("Robots-Memories/Results/info_V4.csv","w" ) -- creating the csv file used to record the outputs corresponding to vehicle4
		file:close()
	elseif IF.myIDS() == 'vehicle5' then
		local file = io.open ("Robots-Memories/Results/info_V5.csv","w" ) -- creating the csv file used to record the outputs corresponding to vehicle5
		file:close()
	elseif IF.myIDS() == 'vehicle6' then
		local file = io.open ("Robots-Memories/Results/info_V6.csv","w" ) -- creating the csv file used to record the outputs corresponding to vehicle6
		file:close()
	elseif IF.myIDS() == 'vehicle7' then
		local file = io.open ("Robots-Memories/Results/info_V7.csv","w" ) -- creating the csv file used to record the outputs corresponding to vehicle7
		file:close()
	elseif IF.myIDS() == 'vehicle8' then
		local file = io.open ("Robots-Memories/Results/info_V8.csv","w" ) -- creating the csv file used to record the outputs corresponding to vehicle8
		file:close()
	elseif IF.myIDS() == 'vehicle9' then
		local file = io.open ("Robots-Memories/Results/info_V9.csv","w" ) -- creating the csv file used to record the outputs corresponding to vehicle9
		file:close()
	elseif IF.myIDS() == 'vehicle10' then
		local file = io.open ("Robots-Memories/Results/info_V10.csv","w" ) -- creating the csv file used to record the outputs corresponding to vehicle10
		file:close()
	elseif IF.myIDS() == 'vehicle11' then
		local file = io.open ("Robots-Memories/Results/info_V11.csv","w" ) -- creating the csv file used to record the outputs corresponding to vehicle11
		file:close()
	end
	-----------	
end

-------------------------------------------------------------------
function reset()
	IF.setTag(IF.myIDS())
	vns:reset()
end

	
	
-------------------------------------------------------------------
--  Main loop for counting robot opinions
-------------------------------------------------------------------
function step()
	The_List_Average = 0
	print("----------" .. IF.myIDS() .. "------------------")
	stepCounter = stepCounter+1 -- step counter

	vns:run{proxTR=IF.getProximityTableTR(),stepCounter = stepCounter, energyN = IF.getenergy()} -- send some parameters to vns module: in this project the 1st and 3rd parameters are not used

	-------------------------------------------------------------------
	-- Initialization
	-------------------------------------------------------------------
	if stepCounter == 1 then  -- initialization
		-- creating a list containg the estimates of all robots and their corresponding time step: initialized with 0 	
		The_List = {vehicle0 = 0, vehicle1 = 0, vehicle2 = 0, vehicle3 = 0, vehicle4 = 0, vehicle5 = 0, vehicle6 = 0, vehicle7 = 0, vehicle8 = 0, vehicle9 = 0, vehicle10 = 0, vehicle11 = 0, info = {v0 = 0, v1 = 0,v2 =  0, v3 =  0, v4 = 0, v5 = 0, v6 = 0, v7 = 0, v8 =  0, v9 = 0, v10 = 0, v11 = 0}}

		for k=1, 2000 do -- creating a 2d vector for storing opinions 
			myOpinions[k] = {}
		end
		myOpinions[stepCounter] = {Average = 0, Voter = 0} -- this two labels ("Average" and "Voter") are not used (later I store the opinion relevant to voter model in an entry label by "p")
		for k=1, 2000 do -- creating an array for storing the total observed blocks by the robot (the relevant values for the most recent 2000 steps) 
			My_Cumulative_BoxCounter[k] = 0
		end
		for k=1, 2000 do -- not used
			Average_Cumulative_BoxCounter[k] = 0 -- not used
		end
	end
	myID = IF.myIDS() -- the ID of the robot
	getBoxCount = IF.getBoxCount() -- read the total number of blocks observed by the robot from the csv file (the csv file is created and updated at each step by the boilerplate_loop_functions.cpp)    
	----------------------------------------------
	if #Average_Cumulative_BoxCounter == 2000 then -- not used
		for k=2, 2000 do
			Average_Cumulative_BoxCounter[k-1] = Average_Cumulative_BoxCounter[k] -- not used
		end
	end
-------------------------------------------------------------------
		
		
	--------------------------------------------------------------------------------------------
	-- Add the number of blocks sensed by each robot in the current step to My_Cumulative_BoxCounter array, and add the corresponding timestamp to MostRecent_Received_Opinion_Time array. 
	-- (Note that each robot maintains one of these arrays.) 
	--------------------------------------------------------------------------------------------
		
	if #My_Cumulative_BoxCounter == 2000 then -- at each step the number of observed blocks by the current step is stored in the last entry of the array and the other values are shifted by one position
		for k=2, 2000 do
			My_Cumulative_BoxCounter[k-1] = My_Cumulative_BoxCounter[k] -- the most 2000 recent recorded "total number of observed blocks" are shifted by one position in the array 
		end
	end
	My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter] = getBoxCount -- for each robot, the number of blocks by the current step is stored in the last entry of the array

	if stepCounter == 1 then -- robot ids used as indices in arrays 
		if IF.myIDS() == 'vehicle0' then
			The_List.info.v0 = stepCounter
			myID_int = 1 
		elseif IF.myIDS() == 'vehicle1' then
			The_List.info.v1 = stepCounter
			myID_int = 2
		elseif IF.myIDS() == 'vehicle2' then
			myID_int = 3
			The_List.info.v2 = stepCounter
		elseif IF.myIDS() == 'vehicle3' then
			myID_int = 4
			The_List.info.v3 = stepCounter
		elseif IF.myIDS() == 'vehicle4' then
			myID_int = 5
			The_List.info.v4 = stepCounter
		elseif IF.myIDS() == 'vehicle5' then
			myID_int = 6
			The_List.info.v5 = stepCounter
		elseif IF.myIDS() == 'vehicle6' then
			myID_int = 7
			The_List.info.v6 = stepCounter
		elseif IF.myIDS() == 'vehicle7' then
			myID_int = 8
			The_List.info.v7 = stepCounter
		elseif IF.myIDS() == 'vehicle8' then
			myID_int = 9
			The_List.info.v8 = stepCounter
		elseif IF.myIDS() == 'vehicle9' then
			myID_int = 10
			The_List.info.v9 = stepCounter
		elseif IF.myIDS() == 'vehicle10' then
			myID_int = 11
			The_List.info.v10 = stepCounter
		elseif IF.myIDS() == 'vehicle11' then
			myID_int = 12
			The_List.info.v11 = stepCounter
		end
			

		-- Notes:
			
		-- each robot keeps My_Cumulative_BoxCounter array of the most recently sensed number of blocks (incl. its own)
		
		-- in MostRecent_Received_Opinion_Time array, each robot recods the timestamp that corresponds with My_Cumulative_BoxCounter array
		-- when the difference between the timestamp and the current time is more than 1000, then the entry for most recent number of blocks is set to 0
		-- initially all the elements of this array are set to 0
			
		for i=1, 12 do -- 8 robots
			MostRecent_Received_Opinion_Time[i] = 0
		end
	end
		
	MostRecent_Received_Opinion_Time[myID_int] = stepCounter -- Set the timestamp corresponding to its most recently sensed density in the MostRecent_Received_Opinion_Time array to the current timestep
	if stepCounter<=1 then -- initialize entry for the most recently sensed density of all robots and their corresponding timestamp in the list with 0; this initialization phase is a redundancy as we did it already when we defined the list
		The_List.vehicle0 = 0
		The_List.info.v0 = 0
		The_List.vehicle1 = 0
		The_List.info.v1 = 0
		The_List.vehicle2 = 0
		The_List.info.v2 = 0
		The_List.vehicle3 = 0
		The_List.info.v3 = 0
		The_List.vehicle4 = 0
		The_List.info.v4 = 0
		The_List.vehicle5 = 0
		The_List.info.v5 = 0
		The_List.vehicle6 = 0
		The_List.info.v6 = 0
		The_List.vehicle7 = 0
		The_List.info.v7 = 0
		The_List.vehicle8 = 0
		The_List.info.v8 = 0
		The_List.vehicle9 = 0
		The_List.info.v9 = 0
		The_List.vehicle10 = 0
		The_List.info.v10 = 0
		The_List.vehicle11 = 0
		The_List.info.v11 = 0
		
	
	elseif stepCounter>1 then -- robot calculates its own most recently sensed density and updates it (with timstamp) in the array
	
			
		-------------------------------------------------------------------
		-- Robot calculates its own estimate and records it (and its timestamp) to The_List
		-------------- Note: Robot estimate is calculated based on the last 1000 time steps
		-------------------------------------------------------------------
		lowerbound = 1 -- this element is 0 during the first 1000 steps
			
		if stepCounter > 1000 then -- only consider the last 1000 timesteps
			lowerbound = #My_Cumulative_BoxCounter-1000
		end
		if IF.myIDS() == 'vehicle0' then
			The_List.vehicle0 = ((My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter] - My_Cumulative_BoxCounter[lowerbound])/(1000*0.11*0.002985))*0.55 -- calculate the estimate based on the last 1000 timesteps and record it to The_List
			The_List.info.v0 = stepCounter -- record the timestamp to The_List

		elseif IF.myIDS() == 'vehicle1' then
			The_List.vehicle1 = ((My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter] - My_Cumulative_BoxCounter[lowerbound])/(1000*0.11*0.002985))*0.55
			The_List.info.v1 = stepCounter

		elseif IF.myIDS() == 'vehicle2' then
			The_List.vehicle2 = ((My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter] - My_Cumulative_BoxCounter[lowerbound])/(1000*0.11*0.002985))*0.55
			The_List.info.v2 = stepCounter

		elseif IF.myIDS() == 'vehicle3' then
			The_List.vehicle3 = ((My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter] - My_Cumulative_BoxCounter[lowerbound])/(1000*0.11*0.002985))*0.55
			The_List.info.v3 = stepCounter

		elseif IF.myIDS() == 'vehicle4' then
			The_List.vehicle4 = ((My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter] - My_Cumulative_BoxCounter[lowerbound])/(1000*0.11*0.002985))*0.55
			The_List.info.v4 = stepCounter

		elseif IF.myIDS() == 'vehicle5' then
			The_List.vehicle5 = ((My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter] - My_Cumulative_BoxCounter[lowerbound])/(1000*0.11*0.002985))*0.55
			The_List.info.v5 = stepCounter

		elseif IF.myIDS() == 'vehicle6' then
			The_List.vehicle6 = ((My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter] - My_Cumulative_BoxCounter[lowerbound])/(1000*0.11*0.002985))*0.55
			The_List.info.v6 = stepCounter

		elseif IF.myIDS() == 'vehicle7' then
			The_List.vehicle7 = ((My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter] - My_Cumulative_BoxCounter[lowerbound])/(1000*0.11*0.002985))*0.55
			The_List.info.v7 = stepCounter

		elseif IF.myIDS() == 'vehicle8' then
			The_List.vehicle8 = ((My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter] - My_Cumulative_BoxCounter[lowerbound])/(1000*0.11*0.002985))*0.55
			The_List.info.v8 = stepCounter

		elseif IF.myIDS() == 'vehicle9' then
			The_List.vehicle9 = ((My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter] - My_Cumulative_BoxCounter[lowerbound])/(1000*0.11*0.002985))*0.55
			The_List.info.v9 = stepCounter

		elseif IF.myIDS() == 'vehicle10' then
			The_List.vehicle10 = ((My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter] - My_Cumulative_BoxCounter[lowerbound])/(1000*0.11*0.002985))*0.55
			The_List.info.v10 = stepCounter

		elseif IF.myIDS() == 'vehicle11' then
			The_List.vehicle11 = ((My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter] - My_Cumulative_BoxCounter[lowerbound])/(1000*0.11*0.002985))*0.55
			The_List.info.v11 = stepCounter

		end
		-------------------------------------------------------------------
		-- Reset the estimate recieved from any robot to 0 (and reset its timestamp to 0), if the estimate is older than 1000 timesteps (in The_List and in MostRecent_Received_Opinion_Time)
		-- Note: there is only one estimate per robot saved to The_List
		-------------------------------------------------------------------				
		for i=1, 12 do -- for all ground robots (here there 12 ground robots)
			if ((MostRecent_Received_Opinion_Time[i] <= stepCounter-1000) and myID_int ~= i) then
					if i == 1 then
						The_List.vehicle0 = 0 -- estimate recieved from vehicle0
						The_List.info.v0 = 0 -- time of estimate recieved from vehicle0
						MostRecent_Received_Opinion_Time[i] = 0 -- time of estimate recieved from vehicle0 (idential to previous line)
					elseif i == 2 then
						The_List.vehicle1 = 0
						The_List.info.v1 = 0
						MostRecent_Received_Opinion_Time[i] = 0
					elseif i == 3 then		
						The_List.vehicle2 = 0
						The_List.info.v2 = 0
						MostRecent_Received_Opinion_Time[i] = 0
					elseif i == 4 then
						The_List.vehicle3 = 0
						The_List.info.v3 = 0
						MostRecent_Received_Opinion_Time[i] = 0
					elseif i == 5 then
						The_List.vehicle4 = 0
						The_List.info.v4 = 0
						MostRecent_Received_Opinion_Time[i] = 0
					elseif i == 6 then
						The_List.vehicle5 = 0
						The_List.info.v5 = 0
						MostRecent_Received_Opinion_Time[i] = 0
					elseif i == 7 then
						The_List.vehicle6 = 0
						The_List.info.v6 = 0
						MostRecent_Received_Opinion_Time[i] = 0
					elseif i == 8 then
						The_List.vehicle7 = 0
						The_List.info.v7 = 0
						MostRecent_Received_Opinion_Time[i] = 0
					elseif i == 9 then
						The_List.vehicle8 = 0
						The_List.info.v8 = 0
						MostRecent_Received_Opinion_Time[i] = 0
					elseif i == 10 then
						The_List.vehicle9 = 0
						The_List.info.v9 = 0
						MostRecent_Received_Opinion_Time[i] = 0
					elseif i == 11 then
						The_List.vehicle10 = 0
						The_List.info.v10 = 0
						MostRecent_Received_Opinion_Time[i] = 0
					elseif i == 12 then
						The_List.vehicle11 = 0
						The_List.info.v11 = 0
						MostRecent_Received_Opinion_Time[i] = 0

					end
			end
		end
		-------------------------------------------------------------------

			
		VNS.Msg.send("broadcast", "List", The_List) -- broadcast The_List to any robot within 1 meter
			
			
		--------------------------------------------------------------------------------------------
		-- Update the estimate recorded for a robot in The_List if the most recent message contains a higher estimate than the currently recorded
		-- Note: If the estimate in the most recent message is the same as the currently recoprded, then update the timestamp if it is more recent
		--------------------------------------------------------------------------------------------

		-- Minor notes:
		-- when robot x receives a message from another robot, it compares each element of the received list with the corresponding element in its own list. 
		-- if the time of the estimate of robot y in the received list is greater than the time of the estimate corresponding to that robot in the robot x's list, robot x overwrites the corresponing time step in its list
			
		for _, msgM in ipairs(vns.Msg.getAM("ALLMSG", "List")) do
			if (myID ~= 'vehicle0' and tonumber(msgM.dataT.vehicle0) ~= nil and  tonumber(msgM.dataT.info.v0) > tonumber(The_List.info.v0)) then
				The_List.vehicle0 = msgM.dataT.vehicle0
				The_List.info.v0 = msgM.dataT.info.v0
				MostRecent_Received_Opinion_Time[1] =  tonumber(msgM.dataT.info.v0)  
			end
			if (myID ~= 'vehicle1' and tonumber(msgM.dataT.vehicle1) ~= nil and tonumber(msgM.dataT.info.v1) > tonumber(The_List.info.v1)) then
				The_List.vehicle1 = msgM.dataT.vehicle1
				The_List.info.v1 = msgM.dataT.info.v1
				MostRecent_Received_Opinion_Time[2] =  tonumber(msgM.dataT.info.v1)  
			end
			if (myID ~= 'vehicle2' and tonumber(msgM.dataT.vehicle2) ~= nil and tonumber(msgM.dataT.info.v2) > tonumber(The_List.info.v2)) then
				The_List.vehicle2 = msgM.dataT.vehicle2
				The_List.info.v2 = msgM.dataT.info.v2
				MostRecent_Received_Opinion_Time[3] =  tonumber(msgM.dataT.info.v2) 
			end
			if (myID ~= 'vehicle3' and tonumber(msgM.dataT.vehicle3) ~= nil and tonumber(msgM.dataT.info.v3) > tonumber(The_List.info.v3)) then
				The_List.vehicle3 = msgM.dataT.vehicle3
				The_List.info.v3 = msgM.dataT.info.v3
				MostRecent_Received_Opinion_Time[4] =  tonumber(msgM.dataT.info.v3) 
			end
			if (myID ~= 'vehicle4' and tonumber(msgM.dataT.vehicle4) ~= nil and tonumber(msgM.dataT.info.v4) > tonumber(The_List.info.v4)) then
				The_List.vehicle4 = msgM.dataT.vehicle4
				The_List.info.v4 = msgM.dataT.info.v4
				MostRecent_Received_Opinion_Time[5] =  tonumber(msgM.dataT.info.v4)  
			end
			if (myID ~= 'vehicle5' and tonumber(msgM.dataT.vehicle5) ~= nil and tonumber(msgM.dataT.info.v5) > tonumber(The_List.info.v5)) then
				The_List.vehicle5 = msgM.dataT.vehicle5
				The_List.info.v5 = msgM.dataT.info.v5
				MostRecent_Received_Opinion_Time[6] =  tonumber(msgM.dataT.info.v5)
			end
			if (myID ~= 'vehicle6' and tonumber(msgM.dataT.vehicle6) ~= nil and tonumber(msgM.dataT.info.v6) > tonumber(The_List.info.v6)) then
				The_List.vehicle6 = msgM.dataT.vehicle6
				The_List.info.v6 = msgM.dataT.info.v6
				MostRecent_Received_Opinion_Time[7] =  tonumber(msgM.dataT.info.v6)
			end
			if (myID ~= 'vehicle7' and tonumber(msgM.dataT.vehicle7) ~= nil and tonumber(msgM.dataT.info.v7) > tonumber(The_List.info.v7)) then
				The_List.vehicle7 = msgM.dataT.vehicle7
				The_List.info.v7 = msgM.dataT.info.v7
				MostRecent_Received_Opinion_Time[8] =  tonumber(msgM.dataT.info.v7) 
			end
			if (myID ~= 'vehicle8' and tonumber(msgM.dataT.vehicle8) ~= nil and  tonumber(msgM.dataT.info.v8) > tonumber(The_List.info.v8)) then
				The_List.vehicle8 = msgM.dataT.vehicle8
				The_List.info.v8 = msgM.dataT.info.v8
				MostRecent_Received_Opinion_Time[9] =  tonumber(msgM.dataT.info.v8) 
			end
			if (myID ~= 'vehicle9' and tonumber(msgM.dataT.vehicle9) ~= nil and tonumber(msgM.dataT.info.v9) > tonumber(The_List.info.v9)) then
				The_List.vehicle9 = msgM.dataT.vehicle9
				The_List.info.v9 = msgM.dataT.info.v9
				MostRecent_Received_Opinion_Time[10] =  tonumber(msgM.dataT.info.v9) 
			end
			if (myID ~= 'vehicle10' and tonumber(msgM.dataT.vehicle10) ~= nil and tonumber(msgM.dataT.info.v10) > tonumber(The_List.info.v10)) then
				The_List.vehicle10 = msgM.dataT.vehicle10
				The_List.info.v10 = msgM.dataT.info.v10
				MostRecent_Received_Opinion_Time[11] =  tonumber(msgM.dataT.info.v10) 
			end
			if (myID ~= 'vehicle11' and tonumber(msgM.dataT.vehicle11) ~= nil and tonumber(msgM.dataT.info.v11) > tonumber(The_List.info.v11)) then
				The_List.vehicle11 = msgM.dataT.vehicle11
				The_List.info.v11 = msgM.dataT.info.v11
				MostRecent_Received_Opinion_Time[12] =  tonumber(msgM.dataT.info.v11) 
			end		

		end -- end of for loop of getting new messages
		--------------------------------------------------------------------------------------------
		
		randomOpinion = {} -- an array used for storing valid estimates of the list, and selecting one of them randomly and considering it as the opinion of the current step in voter model
		ccount = 1 -- a counter to add all valid estimates of the list to randomOpinion array
		MyView_TotalBox = 0 -- a variable to store the summation of all valid estimates of the list
		MyView_TotalBox_Counter = 0 -- a counter to count the number of valid estgimates of the list
		
		
		--------------------------------------------------------------------------------------------
		-- Sum the estimates recorded for each robot in The_List (i.e., one estimate per robot)
		-- Also, copy the estimates recorded for each robot from The_List to randomOpinion
		--------------------------------------------------------------------------------------------
			
		-- Minor notes:
		-- the following if-else statement is used to calculate opinions based on voter and average models
		-- if the time of an estimated stored in the list is greater than timestep-1000 in the MostRecent_Received_Opinion_Time array (i.e., the corresponing estimate has been calculated during the past 1000 steps which means it is valid) add it to the MyView_TotalBox variable
		-- in such a case increase MyView_TotalBox_Counter variable by one, add the corresponding estimate to the randomOpinion array, and increase ccount by one
			
		if The_List.vehicle0 ~= nil and MostRecent_Received_Opinion_Time[1] ~= 0 then
			if (stepCounter<=1000 and MostRecent_Received_Opinion_Time[1] > 0) or (stepCounter>1000 and MostRecent_Received_Opinion_Time[1] > stepCounter-1000) then
				MyView_TotalBox = tonumber(The_List.vehicle0)
				MyView_TotalBox_Counter = MyView_TotalBox_Counter + 1
				randomOpinion[ccount] = tonumber(The_List.vehicle0)
				ccount = ccount + 1

			end
		end
		if The_List.vehicle1 ~= nil and MostRecent_Received_Opinion_Time[2] ~= 0  then
			if (stepCounter<=1000 and MostRecent_Received_Opinion_Time[2] > 0) or (stepCounter>1000 and MostRecent_Received_Opinion_Time[2] > stepCounter-1000) then
				MyView_TotalBox = MyView_TotalBox + tonumber(The_List.vehicle1)
				MyView_TotalBox_Counter = MyView_TotalBox_Counter + 1
				randomOpinion[ccount] = tonumber(The_List.vehicle1)
				ccount = ccount + 1

			end
		end
		if The_List.vehicle2 ~= nil and MostRecent_Received_Opinion_Time[3] ~= 0  then
			if (stepCounter<=1000 and MostRecent_Received_Opinion_Time[3] > 0) or (stepCounter>1000 and MostRecent_Received_Opinion_Time[3] > stepCounter-1000) then
				MyView_TotalBox = MyView_TotalBox + tonumber(The_List.vehicle2)
				MyView_TotalBox_Counter = MyView_TotalBox_Counter + 1
				randomOpinion[ccount] = tonumber(The_List.vehicle2)
				ccount = ccount + 1

			end
		end
		if The_List.vehicle3 ~= nil and MostRecent_Received_Opinion_Time[4] ~= 0  then
			if (stepCounter<=1000 and MostRecent_Received_Opinion_Time[4] > 0) or (stepCounter>1000 and MostRecent_Received_Opinion_Time[4] > stepCounter-1000) then
				MyView_TotalBox = MyView_TotalBox + tonumber(The_List.vehicle3)
				MyView_TotalBox_Counter = MyView_TotalBox_Counter + 1
				randomOpinion[ccount] = tonumber(The_List.vehicle3)
				ccount = ccount + 1

			end
		end
		if The_List.vehicle4 ~= nil and MostRecent_Received_Opinion_Time[5] ~= 0  then
			if (stepCounter<=1000 and MostRecent_Received_Opinion_Time[5] > 0) or (stepCounter>1000 and MostRecent_Received_Opinion_Time[5] > stepCounter-1000) then
				MyView_TotalBox = MyView_TotalBox + tonumber(The_List.vehicle4)
				MyView_TotalBox_Counter = MyView_TotalBox_Counter + 1
				randomOpinion[ccount] = tonumber(The_List.vehicle4)
				ccount = ccount + 1

			end
		end
		if The_List.vehicle5 ~= nil and MostRecent_Received_Opinion_Time[6] ~= 0  then
			if (stepCounter<=1000 and MostRecent_Received_Opinion_Time[6] > 0) or (stepCounter>1000 and MostRecent_Received_Opinion_Time[6] > stepCounter-1000) then
				MyView_TotalBox = MyView_TotalBox + tonumber(The_List.vehicle5)
				MyView_TotalBox_Counter = MyView_TotalBox_Counter + 1
				randomOpinion[ccount] = tonumber(The_List.vehicle5)
				ccount = ccount + 1

			end
		end
		if The_List.vehicle6 ~= nil and MostRecent_Received_Opinion_Time[7] ~= 0  then
			if (stepCounter<=1000 and MostRecent_Received_Opinion_Time[7] > 0) or (stepCounter>1000 and MostRecent_Received_Opinion_Time[7] > stepCounter-1000) then
				MyView_TotalBox = MyView_TotalBox + tonumber(The_List.vehicle6)
				MyView_TotalBox_Counter = MyView_TotalBox_Counter + 1
				randomOpinion[ccount] = tonumber(The_List.vehicle6)
				ccount = ccount + 1

			end
		end
		if The_List.vehicle7 ~= nil and MostRecent_Received_Opinion_Time[8] ~= 0  then
			if (stepCounter<=1000 and MostRecent_Received_Opinion_Time[8] > 0) or (stepCounter>1000 and MostRecent_Received_Opinion_Time[8] > stepCounter-1000) then
				MyView_TotalBox = MyView_TotalBox + tonumber(The_List.vehicle7)
				MyView_TotalBox_Counter = MyView_TotalBox_Counter + 1
				randomOpinion[ccount] = tonumber(The_List.vehicle7)
				ccount = ccount + 1

			end
		end
		if The_List.vehicle8 ~= nil and MostRecent_Received_Opinion_Time[9] ~= 0  then
			if (stepCounter<=1000 and MostRecent_Received_Opinion_Time[9] > 0) or (stepCounter>1000 and MostRecent_Received_Opinion_Time[9] > stepCounter-1000) then
				MyView_TotalBox = MyView_TotalBox + tonumber(The_List.vehicle8)
				MyView_TotalBox_Counter = MyView_TotalBox_Counter + 1
				randomOpinion[ccount] = tonumber(The_List.vehicle8)
				ccount = ccount + 1

			end
		end
		if The_List.vehicle9 ~= nil and MostRecent_Received_Opinion_Time[10] ~= 0  then
			if (stepCounter<=1000 and MostRecent_Received_Opinion_Time[10] > 0) or (stepCounter>1000 and MostRecent_Received_Opinion_Time[10] > stepCounter-1000) then
				MyView_TotalBox = MyView_TotalBox + tonumber(The_List.vehicle9)
				MyView_TotalBox_Counter = MyView_TotalBox_Counter + 1
				randomOpinion[ccount] = tonumber(The_List.vehicle9)
				ccount = ccount + 1

			end
		end
		if The_List.vehicle10 ~= nil and MostRecent_Received_Opinion_Time[11] ~= 0  then
			if (stepCounter<=1000 and MostRecent_Received_Opinion_Time[11] > 0) or (stepCounter>1000 and MostRecent_Received_Opinion_Time[11] > stepCounter-1000) then
				MyView_TotalBox = MyView_TotalBox + tonumber(The_List.vehicle10)
				MyView_TotalBox_Counter = MyView_TotalBox_Counter + 1
				randomOpinion[ccount] = tonumber(The_List.vehicle10)
				ccount = ccount + 1

			end
		end
		if The_List.vehicle11 ~= nil and MostRecent_Received_Opinion_Time[12] ~= 0  then
			if (stepCounter<=1000 and MostRecent_Received_Opinion_Time[12] > 0) or (stepCounter>1000 and MostRecent_Received_Opinion_Time[12] > stepCounter-1000) then
				MyView_TotalBox = MyView_TotalBox + tonumber(The_List.vehicle11)
				MyView_TotalBox_Counter = MyView_TotalBox_Counter + 1
				randomOpinion[ccount] = tonumber(The_List.vehicle11)
				ccount = ccount + 1

			end
		end
		--------------------------------------------------------------------------------------------
			
			
		-------------------------------------------------------------------
		------- AVERAGE MODEL RULE ------- 
		The_List_Average = (MyView_TotalBox/MyView_TotalBox_Counter) -- calculate opinion as the average of the estimates (using the summation above)
		-------------------------------------------------------------------	
				
		Overall_Ave = 0 							-- not used -- initialize the overall opinionan of the average model by 0
		vns.overallAve_Average = vns.overallAve_Average + The_List_Average 	-- not used -- summation of all opinions related to the average model from time step 0 to the current time step
		Overall_Ave = vns.overallAve_Average/stepCounter 			-- not used -- calculate the overall opinion related to the average model by taking average of all the relevant opinions from time step 0 to the current time step	
		print('The result of average over the List: ', MyView_TotalBox/MyView_TotalBox_Counter)
	end
		
		
----------------------------------------------------------------------------
	-- initialization
	if stepCounter==1 then 
		temp = (My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter]/(1000*0.11*0.002985))*0.55 -- calculate the estimate in the first time step
		vns.overallAve_Average = vns.overallAve_Average + temp -- for the overall opinion in the first step (average model)- of course it consider only one step
		Overall_Ave = vns.overallAve_Average/stepCounter  -- overall opinion in the first step (average model)- of course it consider only one step
		vns.overallAve_Randomness = vns.overallAve_Randomness + temp -- for the overall opinion in the first step (average model)- of course it consider only one step
		Overall_Randomness = vns.overallAve_Randomness/stepCounter -- overall opinion in the first step (voter model)- of course it consider only one step

------------------------------creating files--------------------------------
		local file = io.open ("Robots-Memories/Others/ave_1000.csv","r" ) -- used for calculating the summation of opinions (related to the average model) of all the robots in this step
		io.input(file)
		 temp_ave = io.read()


		local file = io.open ("Robots-Memories/Others/ave_1000.csv","w" )
		file:write((tonumber(string.format("%.4f", temp_ave+temp))))
		file:close()


		local file = io.open ("Robots-Memories/Others/rand_1000.csv","r" ) -- used for calculating the summation of opinions (related to the voter model) of all the robots in this step
		io.input(file)
		 temp_rand = io.read()


		local file = io.open ("Robots-Memories/Others/rand_1000.csv","w" )
		file:write(tonumber(string.format("%.4f", temp_rand+temp)))
		file:close()



		local file = io.open ("Robots-Memories/Others/ave_overall.csv","r" ) -- used for calculating the summation of overall opinions (related to the average model) of all the robots in this step
		io.input(file)
		 temp_ave_overall = io.read()


		local file = io.open ("Robots-Memories/Others/ave_overall.csv","w" )
		file:write((tonumber(string.format("%.4f", temp_ave_overall+Overall_Ave))))
		file:close()



		local file = io.open ("Robots-Memories/Others/rand_overall.csv","r" ) -- used for calculating the summation of overall opinions (related to the voter model) of all the robots in this step
		io.input(file)
		 temp_rand_overall = io.read()
		


		local file = io.open ("Robots-Memories/Others/rand_overall.csv","w" )
		file:write((tonumber(string.format("%.4f", temp_rand_overall+Overall_Randomness))))
		file:close()

		if myID == 'vehicle7' then -- in case of 12 robots: if myID == 'vehicle11' then; in case of 4 robots: if myID == 'vehicle3' then
			
			local file = io.open ("Robots-Memories/Others/ave_1000.csv","w" )
			file:write(0)
			file:close()

			local file = io.open ("Robots-Memories/Others/rand_1000.csv","w" )
			file:write(0)
			file:close()

			local file = io.open ("Robots-Memories/Others/ave_overall.csv","w" )
			file:write(0)
			file:close()

			local file = io.open ("Robots-Memories/Others/rand_overall.csv","w" )
			file:write(0)
			file:close()	
		end

		myOpinions[stepCounter].v = 0 -- this label has been added to myOpinions which has been defined earlier but not used later
		myOpinions[stepCounter].p = 0 -- this label has been added to myOpinions which has been defined earlier and has been used to store the opinion of the current step calculated based on the voter model
			
			
------------------------------store the outputs of step 1 into csv files--------------------------------
		local file0 = io.open ("Robots-Memories/Others/info.csv","a" ) -- this csv file is not used
		file0:write(stepCounter.. "\t")
		file0:write(myID.. "\t")
		file0:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
		file0:write(tonumber(string.format("%.4f", temp)).. "\t")
		file0:write(tonumber(string.format("%.4f", temp)).. "\t")
		file0:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
		file0:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
		file0:close()
		
		if myID == 'vehicle0' then -- output of vehicle 0
			local file = io.open ("Robots-Memories/Results/info_V0.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle1' then -- output of vehicle 1
			local file = io.open ("Robots-Memories/Results/info_V1.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle2' then -- output of vehicle 2 
			local file = io.open ("Robots-Memories/Results/info_V2.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle3' then  -- output of vehicle 3 
			local file = io.open ("Robots-Memories/Results/info_V3.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle4' then  -- output of vehicle 4 
			local file = io.open ("Robots-Memories/Results/info_V4.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle5' then  -- output of vehicle 5 
			local file = io.open ("Robots-Memories/Results/info_V5.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle6' then  -- output of vehicle 6 
			local file = io.open ("Robots-Memories/Results/info_V6.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle7' then  -- output of vehicle 7 
			local file = io.open ("Robots-Memories/Results/info_V7.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\t") -- if we use more/less than 8 robots: file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			--the following 4 lines are just for the robot with the biggest ID for getting the average of all the robots' values 
			--in case of 8 robots, they should be done by vehicle7 
			--in case of 12 robots, they should be done by vehicle11
			file:write(tonumber(string.format("%.4f", (temp_ave+temp)/8)).. "\t")
			file:write(tonumber(string.format("%.4f", (temp_rand+temp)/8)).. "\t")
			file:write(tonumber(string.format("%.4f", (temp_ave_overall+Overall_Ave)/8)).. "\t")
			file:write(tonumber(string.format("%.4f", (temp_rand_overall+Overall_Randomness)/8)).. "\n")
			file:close()
		--[[elseif myID == 'vehicle8' then
			local file = io.open ("Robots-Memories/Results/info_V8.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle9' then
			local file = io.open ("Robots-Memories/Results/info_V9.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle10' then
			local file = io.open ("Robots-Memories/Results/info_V10.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle11' then
			local file = io.open ("Robots-Memories/Results/info_V11.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", temp)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\t")
			--the following 4 lines are just for scalability with 12 robots
			--in case of 8 robots, they should be done by vehicle7
			file:write(tonumber(string.format("%.4f", (temp_ave+temp)/12)).. "\t")
			file:write(tonumber(string.format("%.4f", (temp_rand+temp)/12)).. "\t")
			file:write(tonumber(string.format("%.4f", (temp_ave_overall+Overall_Ave)/12)).. "\t")
			file:write(tonumber(string.format("%.4f", (temp_rand_overall+Overall_Randomness)/12)).. "\n")
			file:close()]]
		end
-------------------------------------------------------------------
			
	-- minor implementation details		
	elseif stepCounter>1 then -- for time steps greater than 1

		if #myOpinions == 2000 then -- shift opionins by one postion in the myOpinions 2d vector
			for k=2, 2000 do
				myOpinions[k-1] = myOpinions[k]
			end
		end
			
		
			
			
		-------------------------------------------------------------------
		------- VOTER MODEL RULE ------- 
		rn = math.random(#randomOpinion) -- select random number from the range [1, the length of randomOpinion array]
		myOpinions[#myOpinions].p = randomOpinion[rn] -- select one random estimate from the list of valid estimates stored in randomOpinion array and report it as opinion of the current time step based on the voter model
		-------------------------------------------------------------------
			
		Overall_Randomness = 0 								-- not used -- initialize the overall opinionan of the voter model by 0
		vns.overallAve_Randomness = vns.overallAve_Randomness + randomOpinion[rn] 	-- not used -- summation of all opinions related to the voter model from time step 0 to the current time step
		Overall_Randomness = vns.overallAve_Randomness/stepCounter 			-- not used -- calculate the overall opinion related to the voter model by taking average of all the relevant opinions from time step 0 to the current time step	

		print('myOpinions[stepCounter].Voter: ',myOpinions[#myOpinions].p)
			
			
------------------------------files--------------------------------


		local file = io.open ("Robots-Memories/Others/ave_1000.csv","r" ) -- used for calculating the summation of opinions (related to the average model) of all the robots in this step
		io.input(file)
		 temp_ave = io.read()
		file:close()
		

		local file = io.open ("Robots-Memories/Others/ave_1000.csv","w" )
		file:write((tonumber(string.format("%.4f", temp_ave+The_List_Average))))
		file:close()


		local file = io.open ("Robots-Memories/Others/rand_1000.csv","r" ) -- used for calculating the summation of opinions (related to the voter model) of all the robots in this step 
		io.input(file)
		 temp_rand = io.read()
		file:close()


		local file = io.open ("Robots-Memories/Others/rand_1000.csv","w" )
		file:write(tonumber(string.format("%.4f", temp_rand+myOpinions[#myOpinions].p)))
		file:close()



		local file = io.open ("Robots-Memories/Others/ave_overall.csv","r" )  -- used for calculating the summation of overall opinions (related to the average model) of all the robots in this step 
		io.input(file)
		 temp_ave_overall = io.read()
		file:close()


		local file = io.open ("Robots-Memories/Others/ave_overall.csv","w" )
		file:write((tonumber(string.format("%.4f", temp_ave_overall+Overall_Ave))))
		file:close()



		local file = io.open ("Robots-Memories/Others/rand_overall.csv","r" )  -- used for calculating the summation of overall opinions (related to the voter model) of all the robots in this step
		io.input(file)
		 temp_rand_overall = io.read()
		file:close()
		


		local file = io.open ("Robots-Memories/Others/rand_overall.csv","w" )
		file:write((tonumber(string.format("%.4f", temp_rand_overall+Overall_Randomness))))
		file:close()

		if myID == 'vehicle7' then --in case of 12 robots: if myID == 'vehicle11' then; in case of 4 robots: if myID == 'vehicle3' then
			
			local file = io.open ("Robots-Memories/Others/ave_1000.csv","w" )
			file:write(0)
			file:close()

			local file = io.open ("Robots-Memories/Others/rand_1000.csv","w" )
			file:write(0)
			file:close()

			local file = io.open ("Robots-Memories/Others/ave_overall.csv","w" )
			file:write(0)
			file:close()

			local file = io.open ("Robots-Memories/Others/rand_overall.csv","w" )
			file:write(0)
			file:close()	
		end
------------------------------store the outputs of steps greather than 1 into csv files--------------------------------


		name = IF.myIDS() 
		local file = io.open ("Robots-Memories/Others/info.csv","a" )  -- this csv file is not used
		file:write(stepCounter.. "\t")
		file:write(myID.. "\t")
		file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
		file:write(tonumber(string.format("%.4f", The_List_Average)).. "\t")
		file:write(tonumber(string.format("%.4f", myOpinions[#myOpinions].p)).. "\t")
		file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
		file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
		file:close()
		-----------
		if myID == 'vehicle0' then  -- output of vehicle 0 
			local file = io.open ("Robots-Memories/Results/info_V0.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", The_List_Average)).. "\t")
			file:write(tonumber(string.format("%.4f", myOpinions[#myOpinions].p)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle1' then  -- output of vehicle 1 
			local file = io.open ("Robots-Memories/Results/info_V1.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", The_List_Average)).. "\t")
			file:write(tonumber(string.format("%.4f", myOpinions[#myOpinions].p)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle2' then  -- output of vehicle 2 
			local file = io.open ("Robots-Memories/Results/info_V2.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", The_List_Average)).. "\t")
			file:write(tonumber(string.format("%.4f", myOpinions[#myOpinions].p)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle3' then  -- output of vehicle 3 
			local file = io.open ("Robots-Memories/Results/info_V3.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", The_List_Average)).. "\t")
			file:write(tonumber(string.format("%.4f", myOpinions[#myOpinions].p)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle4' then  -- output of vehicle 4 
			local file = io.open ("Robots-Memories/Results/info_V4.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", The_List_Average)).. "\t")
			file:write(tonumber(string.format("%.4f", myOpinions[#myOpinions].p)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle5' then  -- output of vehicle 5 
			local file = io.open ("Robots-Memories/Results/info_V5.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", The_List_Average)).. "\t")
			file:write(tonumber(string.format("%.4f", myOpinions[#myOpinions].p)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle6' then  -- output of vehicle 6 
			local file = io.open ("Robots-Memories/Results/info_V6.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", The_List_Average)).. "\t")
			file:write(tonumber(string.format("%.4f", myOpinions[#myOpinions].p)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle7' then  -- output of vehicle 7 
			local file = io.open ("Robots-Memories/Results/info_V7.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", The_List_Average)).. "\t")
			file:write(tonumber(string.format("%.4f", myOpinions[#myOpinions].p)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\t") -- if we use more/less than 8 robots: file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			--the following 4 lines are just for the robot with the biggest ID for getting the average of all the robots' values
			--in case of 8 robots, they should be done by vehicle7
			--in case of 12 robots, they should be done by vehicle11
			file:write(tonumber(string.format("%.4f", (temp_ave+The_List_Average)/8)).. "\t")
			file:write(tonumber(string.format("%.4f", (temp_rand+myOpinions[#myOpinions].p)/8)).. "\t")
			file:write(tonumber(string.format("%.4f", (temp_ave_overall+Overall_Ave)/8)).. "\t")
			file:write(tonumber(string.format("%.4f", (temp_rand_overall+Overall_Randomness)/8)).. "\n")
			file:close()
		--[[elseif myID == 'vehicle8' then  -- output of vehicle 8 
			local file = io.open ("Robots-Memories/Results/info_V8.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", The_List_Average)).. "\t")
			file:write(tonumber(string.format("%.4f", myOpinions[#myOpinions].p)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle9' then  -- output of vehicle 9 
			local file = io.open ("Robots-Memories/Results/info_V9.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", The_List_Average)).. "\t")
			file:write(tonumber(string.format("%.4f", myOpinions[#myOpinions].p)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle10' then  -- output of vehicle 10 
			local file = io.open ("Robots-Memories/Results/info_V10.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", The_List_Average)).. "\t")
			file:write(tonumber(string.format("%.4f", myOpinions[#myOpinions].p)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\n")
			file:close()
		elseif myID == 'vehicle11' then  -- output of vehicle 11 
			local file = io.open ("Robots-Memories/Results/info_V11.csv","a" )
			file:write(stepCounter.. "\t")
			file:write(My_Cumulative_BoxCounter[#My_Cumulative_BoxCounter].. "\t")
			file:write(tonumber(string.format("%.4f", The_List_Average)).. "\t")
			file:write(tonumber(string.format("%.4f", myOpinions[#myOpinions].p)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Ave)).. "\t")
			file:write(tonumber(string.format("%.4f", Overall_Randomness)).. "\t")
			--the following 4 lines are just for scalability with 12 robots
			--in case of 8 robots, they should be done by vehicle7
			file:write(tonumber(string.format("%.4f", (temp_ave+The_List_Average)/12)).. "\t")
			file:write(tonumber(string.format("%.4f", (temp_rand+myOpinions[#myOpinions].p)/12)).. "\t")
			file:write(tonumber(string.format("%.4f", (temp_ave_overall+Overall_Ave)/12)).. "\t")
			file:write(tonumber(string.format("%.4f", (temp_rand_overall+Overall_Randomness)/12)).. "\n")
			file:close()]]
		end
		-----------
	end

end

-------------------------------------------------------------------
function destroy()
end

------------------------------------------------------------------------
---- this section not used	
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

-- the ground robot moves by calling this function
-- the speed of the ground robot is always 7.46 cm/s when it is moving forward
-- the speed of the left wheel of the ground robot and the speed of the right wheel of it are always +7.46 cm/s and -7.46 cm/s when it is turing right
-- the speed of the left wheel of the ground robot and the speed of the right wheel of it are always -7.46 cm/s and +7.46 cm/s when it is turing left	
VNS.move = function(transV3, rotateV3,stepCounter, energy) -- this funtion makes the robot to move and is called by Drive.lua file
------------------------------------------------------------------------
		
		
		
	------------------------------------------------------------------------
	-- Handling of all motion vectors in the decentralized approaches 	
	------------------------------------------------------------------------
	local left = transV3.x -- take x of transV3 (a 3D vector used to indicate motion control) 
	local right = transV3.x -- take x of transV3 (a 3D vector used to indicate motion control) 

	local turnRate = 1
	local speedscale = 15

	local smalllimit = 0.3 -- used to set a minimum magnitude of the vector
	-- ensure the vector has a large enough magnitude for the robot
	if 0 <= transV3.x and transV3.x < smalllimit then transV3.x = smalllimit end
	if 0 > transV3.x and transV3.x >-smalllimit then transV3.x =-smalllimit end
		
	-- define left and right	
	left  = left  - transV3.y/transV3.x * turnRate -- set as: x - (y/x)
	right = right + transV3.y/transV3.x * turnRate -- set as: x + (y/x)
	
	-- multiply both right and left by the same value	
	left = left * speedscale
	right = right * speedscale

	if state == 0 then -- state is always 0 in this project
			
		-- set the speeds for the left and right motor
		-- possible outcomes are: 
			-- left wheel pos, right wheel neg (i.e., robot turns right at a constant radial velocity of 7.46 cm/s)
			-- left wheel neg, right wheel pos (i.e., robot turns left at a constant radial velocity of 7.46 cm/s)
			-- left wheel pos, right wheel pos (i.e., robot move forward at a constant linear velocity of 7.46 cm/s)
			
		if left > 0 then  -- if (x - (y/x)) is positve, then set the left wheel to turn forward at the constant radial velocity (7.46 cm/s)
			left = 7.46
		end
		if right > 0 then -- if (x + (y/x)) is positve, then set the right wheel to turn forward at the constant radial velocity (7.46 cm/s)
			right = 7.46
		end
		if left < -0 then -- if (x - (y/x)) is negative, then set the left wheel to turn backward at the constant radial velocity (7.46 cm/s)
			left = -7.46
		end
		if right < -0 then  -- if (x + (y/x)) is negative, then set the right wheel to turn backward at the constant radial velocity (7.46 cm/s)
			right = -7.46
		end
		
	end 
	IF.setVelocity(left, right)
	
end

function IF.setenergy(m) -- not used
	energy = math.abs(m)
	local file = io.open ("energy.csv","w" )
	file:write(energy)
	file:close()
end

function IF.getenergy() -- not used
	local file = io.open ("energy.csv","r" )
	io.input(file)
	local energy = io.read()
	return energy
end

function IF.getBoxCount() -- read the total number of blocks observed by the robot from the csv file (the csv file is created and updated at each step by the boilerplate_loop_functions.cpp) 
	name = IF.myIDS() 
	local file = io.open ("Robots-Memories/Others/"..name..".csv","r" )
	io.input(file)
	local energy = io.read()
	return energy
end

VNS.setSpeed = function(x, y)
	IF.setVelocity(x, -y)
end
------------------------------------------------------------------------
--  Robot Interface 
------------------------------------------------------------------------
function IF.getProximityTableTR() --not used
	return robot.proximity
end

function IF.getProximityN(x) -- not used
	return robot.proximity[x]
end

function IF.myIDS() -- return the robot's ID
	return robot.id
end

function IF.setTag(str)
	robot.tags.set_all_payloads(str)
end

function IF.setVelocity(x, y)
	robot.joints.base_wheel_left.set_target(x)
	robot.joints.base_wheel_right.set_target(-y)
end
