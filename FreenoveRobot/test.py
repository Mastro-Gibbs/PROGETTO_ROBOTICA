from virtual_body import VirtualBody, BodyException
from time import sleep


if __name__ == "__main__":
    virtual_body = None
    try:
        virtual_body = VirtualBody()

        while not virtual_body.begin():
            print('Missing component, retrying, delay 5 seconds')
            sleep(5)

        print('\nVirtualBody ready\nIgnore possible buried threads messages')
        virtual_body.loop()
    except KeyboardInterrupt:
        pass
    except BodyException as be:
        print(be.args[0])
    finally:
        if virtual_body:
            virtual_body.stop()