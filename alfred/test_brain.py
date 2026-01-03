from alfred.brain import process_text
from shared_consts import NAMED_POSES

# --- TEST RUNNER ---
if __name__ == "__main__":
    test_commands = {
        "move to the up position": NAMED_POSES['up'],      # Should match "up" exactly
        "look to the left":  NAMED_POSES['left'],              # Should match "left"
        "go strictly right":  NAMED_POSES['right'],             # Should match "right"
        "please crouch down":  NAMED_POSES['crouch'],            # Should match "crouch"
        "jibberish banana sandwich":  NAMED_POSES['crouch'],     # Should fail -> Fallback to Crouch
        "":  NAMED_POSES['crouch'],                              # Empty -> Fallback to Crouch
    }

    print("--- STARTING BRAIN TEST ---\n")
    for cmd in test_commands:
        print(f"Testing Command: '{cmd}'")
        result = process_text(cmd)
        print(f"Resulting Joints: {result}")
        assert result ==test_commands[cmd], "FAILED"
        print("-" * 30)
