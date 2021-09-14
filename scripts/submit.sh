#!/bin/bash

set -o errexit

unset username
unset pass
unset description

printf "This script will compile your project & upload to Riders.ai for evaluation.\n"
printf "Please enter your Riders.ai Username & Password\n\n"

echo -n "Username:"
read username
echo -n "Password:"
read -s pass

SCRIPT_DIR=$(dirname "$0")

cd $SCRIPT_DIR/..

set -a # automatically export all variables
source ./.env
set +a

printf "\nRetrieving your API Token...\n"

RED='\033[0;31m'
NC='\033[0m' # No Color

DATA='{"username": '\""$username"\"', "password": '\""$pass"\"'}'

COLAB_HOST="$RIDERS_API_HOST"'/get-token/'

COLAB_TOKEN_RESPONSE=$(curl -H "Content-Type: application/json" -X POST -d "$DATA" "$COLAB_HOST")

STATUS_CODE=$(curl --write-out '%{http_code}' --silent --output /dev/null -H "Content-Type: application/json" -X POST -d "$DATA" "$COLAB_HOST")

if [ $STATUS_CODE == "404" ]; then
  echo -e "${RED}Authorization Failed: Token Not Found"
  exit 1
elif [ $STATUS_CODE == "403" ]; then
  echo "${RED}Too Many Attemps, Please contact with Riders.ai admins"
  exit 1
elif [ $STATUS_CODE == "200" ]; then
  echo "Authorization Successful"
fi

RIDERS_AUTH_TOKEN=$(echo $COLAB_TOKEN_RESPONSE | jq -c '.token' | tr -d '"')
if [[ -z ${RIDERS_AUTH_TOKEN} ]]; then
  printf "Faulty Auth Token, please try restarting this script.\n"
  exit
fi

# TODO: bir onceki satir eger hata donerse exit code, none 0 olmali, if so display error

printf "\n\nBuilding your Agent Docker image, this may take a while...\n"

# Try to build agent locally to prevent sending user_projects that fail on build process.
docker-compose build agent

# We assume this script was placed on the project root. Otherwise we should change directory.
# Archive tar file with arbitrary filename
printf "\n\nBuilding TAR file, this may take a while...\n"
git ls-files | tar Tzcf - challengeproject.tar.gz
printf "Compiled TAR file for submission.\n"

printf "\n\nPlease enter a short description of your submission. This would help you compare separate submissions\n"
echo -n "Description:"
read description

RIDERS_API_SUBMIT="$RIDERS_RIDERS_API_HOST"'/challenge/'"$RIDERS_CHALLENGE_ID"'/submit'

# TODO: Fix arbitrary constants
curl --location \
    -F 'file=@./challengeproject.tar.gz' \
    -F description="$description" \
    -H "Authorization: TOKEN'" $RIDERS_AUTH_TOKEN"
    "$RIDERS_API_SUBMIT"

