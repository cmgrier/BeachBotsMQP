from baseBot.CleaningManager import CleaningManager
from data.Robot import Robot


def test_create_cleaning_zones():
    print()


# for testing
if __name__ == '__main__':
    robots = [Robot(0)]
    cleaningManager = CleaningManager(robots)
    test_create_cleaning_zones()
    cleaningManager.mapManager.zones
