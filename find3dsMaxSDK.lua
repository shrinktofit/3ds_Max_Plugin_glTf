function find()
    adsk3dsMaxHomeTests = {"ADSK_3DSMAX_SDK_2020", "ADSK_3DSMAX_SDK_2019"}
    for iTest = 1, #adsk3dsMaxHomeTests do
        test = adsk3dsMaxHomeTests[iTest]
        testResult = os.getenv(test)
        if testResult ~= nil then
            -- print('[' .. iTest .. ']' .. 'Found ' .. test .. ': ' .. testResult)
            return testResult
        else
		    -- print('[' .. iTest .. ']' .. test .. " doesn't exists.")
        end
    end
end
