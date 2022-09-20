function showTable(table, number, dontShowS)
	if number == nil then number = 0 end
	if type(table) ~= "table" then return nil end

	local xyz = nil
	if table.x ~= nil and table.y ~= nil and table.z ~= nil then
		xyz = "x"
	end

	local flag = false
	for i, v in pairs(table) do
		flag = true
		if i == "x" or i == "y" or i == "z" then
			if xyz == "x" then
				i = "x"  v = table.x
				xyz = "y"
			elseif xyz == "y" then
				i = "y"  v = table.y
				xyz = "z"
			elseif xyz == "z" then
				i = "z"  v = table.z
				xyz = nil
			end
		end

		if i ~= dontShowS then

		local str = ""
		for j = 1, number do
			str = str .. "\t"
		end

		str = str .. tostring(i) .. "\t"

		if type(v) == "table" and v.CLASSQUATERNION == true then
			local ang = v:getAng()
			if v:getAxis().z < 0 then ang = -ang end
			str = str .. tostring(ang*180/math.pi)
			print(str)
		elseif type(v) == "table" then
			print(str)
			showTable(v, number + 1, dontShowS)
		else
			str = str .. tostring(v)
			print(str)
		end
	end end
	if flag == false then
		local str = ""
		for j = 1, number do
			str = str .. "\t"
		end

		print(str .. "EMPTYTABLE")
	end
end

