

serial_instance = serial:find_serial(0)

serial_instance:begin(19200)

function update()
    local imei_command = "AT+CGSN\r"
    serial_instance:writestring(imei_command)

    local datacount = serial_instance:available()

    if datacount > 0 then
        local received_data = serial_instance:readstring(datacount:toint())
        gcs:send_text(4, "iridium: " .. received_data)
    end

    return update, 100
end

return update, 1000