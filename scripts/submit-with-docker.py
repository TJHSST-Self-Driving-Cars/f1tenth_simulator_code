import docker
import os

SCRIPTS_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.dirname(SCRIPTS_DIR)
SUBMIT_DOCKERFILE_PATH = os.path.join(PROJECT_ROOT, 'compose', 'submit', 'Dockerfile')
AGENT_DOCKERFILE_PATH = os.path.join(PROJECT_ROOT, 'compose', 'agent', 'Dockerfile')
AGENT_PARENT = "registry.gitlab.com/acrome-colab/riders-poc/f1tenth-riders-quickstart/roscore:latest"


def build_agent_image():
    client = docker.from_env()
    client.build(
        path=PROJECT_ROOT,
        dockerfile=AGENT_DOCKERFILE_PATH,
        tag="a1",
        buildargs={
            "PARENT": AGENT_PARENT
        }
    )


def build_submit_image():
    client = docker.from_env()
    client.build(
        path=PROJECT_ROOT,
        dockerfile=SUBMIT_DOCKERFILE_PATH,
        tag="registry.gitlab.com/acrome-colab/riders-poc/f1tenth-riders-quickstart/submit",
    )


if __name__ == '__main__':
    print("Building your Agent Docker image, this may take a while...")
    build_agent_image()

    input("")

#
#
# set -o errexit
#
# printf "\n\nBuilding your Agent Docker image, this may take a while...\n"
#
# # Try to build agent locally to prevent sending user_projects that fail on build process.
# docker-compose -f docker-compose.yml build agent
#
# unset RIDERS_USERNAME
# unset RIDERS_PASSWORD
# unset DESCRIPTION
#
# printf "Image built & tagged successfully.\n"
# printf "\n\nThis script will compile your project & upload to Riders.ai for evaluation.\n"
# printf "Please enter your Riders.ai Username & Password\n\n"
#
# echo -n "Username:"
# read RIDERS_USERNAME
# echo -n "Password:"
# read -s RIDERS_PASSWORD
#
# printf "\n\nPlease enter a short description of your submission. This would help you compare separate submissions\n"
# echo -n "Description:"
# read DESCRIPTION
#
# export RIDERS_USERNAME
# export RIDERS_PASSWORD
# export DESCRIPTION
# docker-compose -f scripts/submit-docker-compose.yml build submit
# docker-compose -f scripts/submit-docker-compose.yml up submit
#
# printf "Submission successful! Please go to https://riders.ai/challenge/40/submissions to validate."
