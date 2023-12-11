import pandas as pd
from pandas import DataFrame


def get_raw_data(path_to_data: str = "data/simulation_data.csv") -> DataFrame:
    return pd.read_csv(path_to_data)


def process_data(data: DataFrame) -> tuple[DataFrame, DataFrame]:
    # TODO
    # should return (X, y)
    raise NotImplementedError


if __name__ == "__main__":
    print(get_raw_data("implementations/ai/data/example_data.csv"))
