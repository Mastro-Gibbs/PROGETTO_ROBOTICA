import physical_body

p = physical_body.PhysicalBody()


def loop():
    while True:
        p.send_perceptions()
        p.get_commands()


loop()
