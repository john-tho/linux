////////////////////////////////////////////////
//service area, skip to "developers area" below
@Library('utils_static') _
node("master") {
    utils_static.loadLibraries()
}
////////////////////////////////////////////////
//developers area
class Globals {
   static String COMPILER_PREFIX_FOR_COVERITY = ""
   static String GIT_PROJECT_NAME = "HAL"
}
//build andeckers properties
def git_URL = "${GERRIT_SSH}/${Globals.GIT_PROJECT_NAME}"

def build_HAL_command = "./devops/ci_scripts/jenkins_HAL_build.sh"
def static_analysis_command = "./devops/ci_scripts/jenkins_static_analysis_check.sh"
def doxygen_check_command = "./devops/ci_scripts/jenkins_doxygen_check.sh"

def checkpatch_jobs = [:]
def build_jobs = [:]

if (!(utils.decode_gerrit_comment().contains("JENKINS_WAIVER_CHECK_PATCH")) &&
    !(utils.decode_gerrit_commit_message().contains("JENKINS_WAIVER_CHECK_PATCH"))) {
    checkpatch_jobs['CHECKPATCH'] = {
        utils.build_ci_flow(git_URL, "dvp_builder",
			    "./check_commit_message.pl; ./checkpatch/check_last_patch.sh")
    }
}
if (!(utils.decode_gerrit_comment().contains("JENKINS_WAIVER_BUILD_HAL")) &&
    !(utils.decode_gerrit_commit_message().contains("JENKINS_WAIVER_BUILD_HAL"))) {
    build_jobs['BUILD_HAL'] = {
        utils.build_ci_flow(git_URL, "dvp_builder", build_HAL_command)
    }
}
if (!(utils.decode_gerrit_comment().contains("JENKINS_WAIVER_CHECK_SPARSE")) &&
    !(utils.decode_gerrit_commit_message().contains("JENKINS_WAIVER_CHECK_SPARSE"))) {
    build_jobs['STATIC_ANALYSIS'] = {
        utils.build_ci_flow(git_URL, "dvp_builder", static_analysis_command)
    }
}
if (!(utils.decode_gerrit_comment().contains("JENKINS_WAIVER_CHECK_DOXYGEN")) &&
    !(utils.decode_gerrit_commit_message().contains("JENKINS_WAIVER_CHECK_DOXYGEN"))) {
    build_jobs['DOXYGEN'] = {
        utils.build_ci_flow(git_URL, "dvp_builder", doxygen_check_command)
    }
}

try {
//build and checkers
    parallel checkpatch_jobs
    parallel build_jobs
} catch (err) {
    utils.top_level_error_handler(err)
}
finally {
    // ending the pipe when all the previous steps are done
    // email reporting, test result parsing, etc. happens here:
    utils.wrap_up_flow ()
}
