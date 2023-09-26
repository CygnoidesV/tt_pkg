import yaml

CONFIG_PATH = "/home/user/TransportTrolley/src/tt_pkg/data/config.yaml"

class Config:
    def __init__(self) -> None:
        self.data = {}
        try:
            with open(CONFIG_PATH, 'r') as file:
                self.data = yaml.safe_load(file)
        except FileNotFoundError:
            print(f"Config file was not found. Please make sure the path '{CONFIG_PATH}' is correct.")
        except yaml.YAMLError as e:
            print(f"Error reading YAML from '{CONFIG_PATH}': {str(e)}")
        except Exception as e:
            print(f"An error occurred while reading the file '{CONFIG_PATH}': {str(e)}")

    def get(self, key):
        return self.data[key]


config = Config()

settings_BL = config.data["settings_BL"]
settings_PU = config.data["settings_PU"]

if __name__ == "__main__":
    # 输出参数信息
    print("YAML Parameters:")
    print(settings_BL)
