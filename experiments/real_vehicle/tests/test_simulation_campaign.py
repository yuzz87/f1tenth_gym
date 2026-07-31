import copy
import csv
import tempfile
import unittest
from pathlib import Path

from experiments.real_vehicle.evaluation.common import load_real_config
from experiments.real_vehicle.evaluation.simulation_campaign import (
    SimulationChannel,
    expand_cases,
    load_campaign,
    run_case,
)
from experiments.real_vehicle.evaluation.simulation_report import generate_report


class SimulationCampaignTest(unittest.TestCase):
    def test_channel_outage_and_recovery(self):
        channel = SimulationChannel({
            "control_outage_after_s": 1.0,
            "control_outage_duration_s": 0.5,
        }, prefix="control_")
        channel.send("before", 0.0)
        channel.send("during", 1.2)
        channel.send("after", 1.6)
        self.assertEqual(channel.receive(2.0), ["before", "after"])
        self.assertEqual(channel.statistics.outage_dropped, 1)

    def test_default_campaign_expands_all_dimensions(self):
        campaign = load_campaign()
        expected = 3 * 2 * 2 * 2 * 3 * 2 * 4
        self.assertEqual(len(expand_cases(campaign)), expected)

    def test_short_case_and_report_generation(self):
        campaign = load_campaign()
        config, parameters, limits = load_real_config()
        case = {
            "seed": 123,
            "controller": "mpc",
            "scenario": "straight",
            "pose_source": "ground_truth",
            "plant_profile": "nominal",
            "lidar_profile": "ideal",
            "network_profile": "normal",
        }
        summary = run_case(
            case,
            copy.deepcopy(config),
            campaign,
            parameters,
            limits,
            max_steps=10,
        )
        self.assertEqual(summary["steps"], 10)
        self.assertGreater(summary["lidar_valid_ratio"], 0.0)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            runs = output / "runs.csv"
            with runs.open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file, fieldnames=list(summary))
                writer.writeheader()
                writer.writerow(summary)
            report = generate_report(runs, output)
            self.assertTrue(report.exists())
            self.assertTrue((output / "summary.json").exists())


if __name__ == "__main__":
    unittest.main()
